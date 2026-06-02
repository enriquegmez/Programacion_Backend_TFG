"""
ros_node_handler.py
Enlace entre el servidor asíncrono y el ecosistema de ROS 2.
Maneja el ciclo de vida de rclpy en un hilo dedicado y publica en /cmd_vel.
"""

import logging
import threading
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor

from app_tiago.utils.constants import ControlEvent
from app_tiago.ros.ros2_core_node import SafetyFilterNode

class TiagoBridgeNode(Node):
    def __init__(self):
        super().__init__('app_tiago_bridge')
        self.logger = logging.getLogger("TiagoBridgeNode")
        
        # Publicador real al tópico de velocidad de Tiago
        self.vel_publisher = self.create_publisher(msg_type=Twist, 
                                                   topic='web_teleop/cmd_vel_raw', 
                                                   qos_profile=10)
        self.logger.info("Puente ROS 2 iniciado. Publicando raw en: /web_teleop/cmd_vel_raw")

        # Estado de seguridad
        self.is_connected = False
        self.is_control_active = False

    def connect(self) -> bool:
        """Verifica que existan nodos del robot conectados a la red ROS 2."""
        # Obtenemos todos los nombres de los nodos activos en la red
        nodos_activos = self.get_node_names()
        
        # Excluimos nuestros propios nodos de la lista
        nodos_nuestros = ['app_tiago_bridge', 'app_safety_filter']
        nodos_robot = [nodo for nodo in nodos_activos if nodo not in nodos_nuestros]
        
        # Si la lista está vacía, no hay ningún robot simulado o real encendido
        if len(nodos_robot) == 0:
            self.logger.warning("Conexión rechazada: No se ha detectado ningún nodo del robot en la red (¿Simulador apagado?).")
            return False
            
        self.is_connected = True
        self.logger.info(f"Robot detectado en la red. (Nodos ajenos encontrados: {len(nodos_robot)})")
        return True

    def disconnect(self):
        """Frena el robot y rompe la conexión lógica."""
        if self.is_connected:
            self.logger.info("Desconectando del Tiago. Aplicando freno de emergencia.")
            self.stop_robot()
            self.is_connected = False
            self.is_control_active = False

    def check_connection_silently(self) -> bool:
        """Comprueba si el robot sigue ahí sin saturar los logs."""
        if not self.is_connected:
            return False
            
        nodos_activos = self.get_node_names()
        nodos_nuestros = ['app_tiago_bridge', 'app_safety_filter']
        nodos_robot = [nodo for nodo in nodos_activos if nodo not in nodos_nuestros]
        
        return len(nodos_robot) > 0

    # ==========================================
    # ¡NUEVO! COMPROBACIÓN DEL SERVIDOR DE VÍDEO
    # ==========================================
    def check_video_server_silently(self) -> bool:
        """Busca el nodo web_video_server en la red."""
        try:
            nodos_activos = self.get_node_names()
            return 'web_video_server' in nodos_activos
        except Exception as e:
            self.logger.error(f"Error buscando el servidor de vídeo: {e}")
            return False
        
    def validate_topic(self, topic_name: str) -> tuple[bool, str]:
        """Comprueba en la red ROS 2 si el topic existe y es de tipo Twist."""
        # Limpiamos el nombre (por si llega con espacios o barras raras)
        clean_topic = topic_name.strip()
        
        # Pedimos a la red la lista de TODOS los tópicos actuales
        topics_and_types = self.get_topic_names_and_types()
        
        for name, types in topics_and_types:
            # Comparamos (name == clean_topic) o si está bajo un namespace (name.endswith)
            if name == clean_topic or name.endswith(f'/{clean_topic.lstrip("/")}') or name == f'/{clean_topic}':
                if 'geometry_msgs/msg/Twist' in types:
                    # ARREGLO ERROR 3: Devolvemos el nombre ABSOLUTO encontrado ('name')
                    return True, name
                else:
                    return False, f"El topic '{name}' existe, pero no acepta Twist. Usa: {types[0]}"
                    
        return False, f"El topic '{clean_topic}' no se ha encontrado en la red del robot."
    
    def set_control_mode(self, event: str, topic: str = "cmd_vel") -> tuple[bool, str]:
        """Activa/Desactiva el control devolviendo (Éxito, Mensaje)."""
        if not self.is_connected:
            self.logger.error("Cambio de modo denegado: Robot no conectado.")
            return False, "Robot no conectado lógicamente."

        if event == ControlEvent.START:
            # 1. VALIDAMOS EL TOPIC EN CALIENTE
            is_valid, msg = self.validate_topic(topic)
            if not is_valid:
                self.logger.warning(f"Validación de topic fallida: {msg}")
                return False, msg

            self.is_control_active = True
            self.logger.info(f"Control de Joystick ACTIVADO. Topic validado: {msg}")
            return True, msg
            
        elif event == ControlEvent.STOP:
            self.is_control_active = False
            self.logger.info("Control de Joystick DESACTIVADO. Frenando robot.")
            self.stop_robot()
            return True, ""
            
        return False, "Evento desconocido."

    def publish_velocity(self, v: float, w: float) -> bool:
        """Envía el comando Twist si se cumplen las condiciones de seguridad."""
        if not self.is_connected or not self.is_control_active:
            # Devolvemos False pero no hacemos log de error para no saturar la consola
            # si el usuario mueve el joystick cuando no debe (a 50Hz)
            return False

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        
        self.vel_publisher.publish(msg)
        return True

    def stop_robot(self):
        """Freno por software: Velocidad 0 en todos los ejes."""
        try:
            # Comprobamos que el núcleo de ROS 2 no se haya destruido en otro hilo
            if rclpy.ok() and self.vel_publisher is not None:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.vel_publisher.publish(msg)
        except Exception as e:
            # Si el publicador explota por culpa del Ctrl+C, lo ignoramos en silencio
            self.logger.debug(f"Freno omitido por cierre repentino de contexto: {e}")


class Ros2Manager:
    """
    Controlador del hilo de ROS 2. Aísla rclpy.spin() del bucle asyncio del servidor web.
    """
    def __init__(self):
        self.logger = logging.getLogger("Ros2Manager")
        self.gateway_node = None
        self.safety_node = None
        self.executor = None
        self.spin_thread = None
        self._is_running = False

    def start(self):
        if self._is_running:
            return

        self.logger.info("Arrancando subsistema ROS 2 (Gateway + Filtro)...")
        rclpy.init()
        
        # 1. Instanciamos AMBOS nodos
        self.gateway_node = TiagoBridgeNode()
        self.safety_node = SafetyFilterNode()
        
        # 2. Creamos el Ejecutor y le añadimos los nodos
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.gateway_node)
        self.executor.add_node(self.safety_node)
        
        self._is_running = True

        # 3. Arrancamos el hilo usando el ejecutor en lugar del nodo
        self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self.spin_thread.start()

    def _spin_loop(self):
        try:
            # spin() en el ejecutor procesa los eventos de TODOS los nodos que contenga
            self.executor.spin()
        except Exception as e:
            self.logger.error(f"Error crítico en rclpy executor: {e}")
        finally:
            self.logger.info("Hilo de ROS 2 finalizado.")

    def stop(self):
        if not self._is_running:
            return

        self.logger.info("Apagando subsistema ROS 2...")
        self._is_running = False
        
        try:
            if self.gateway_node:
                self.gateway_node.disconnect()

                # --- ¡EL ARREGLO MÁGICO! ---
            # Le decimos al filtro de seguridad que envíe el freno 
            # directamente al robot, saltándose las comunicaciones intermedias.
            if self.safety_node:
                self.safety_node.stop_robot()
            
            # Damos 0.2 segundos a la red DDS para enviar el freno antes de apagar
            time.sleep(0.2)
        except Exception:
            pass
            
        try:
            if self.executor:
                self.executor.shutdown()
                
            if self.gateway_node:
                self.gateway_node.destroy_node()
            if self.safety_node:
                self.safety_node.destroy_node()
                
            if rclpy.ok():
                rclpy.shutdown()
                
            if self.spin_thread:
                self.spin_thread.join(timeout=2.0)
        except Exception as e:
            self.logger.error(f"Ignorando error durante el cierre forzado de ROS 2: {e}")

    # ==========================================
    # API PÚBLICA PARA EL ROUTER
    # ==========================================
    def connect_to_robot(self) -> bool:
        if self._is_running and self.gateway_node:
            return self.gateway_node.connect()
        return False

    def disconnect_from_robot(self):
        if self._is_running and self.gateway_node:
            self.gateway_node.disconnect()
        
    def stop_robot(self):
        """Freno de emergencia expuesto para el Router."""
        if self._is_running and self.gateway_node:
            self.gateway_node.stop_robot()

    def set_control_mode(self, event: str, topic: str = "cmd_vel") -> tuple[bool, str]:
        if self._is_running and self.gateway_node and self.safety_node:
            # Ahora recogemos el success y el mensaje de error
            success, msg = self.gateway_node.set_control_mode(event, topic)
            
            if success and event == ControlEvent.START:
                self.safety_node.set_target_topic(msg)
                
            return success, msg
            
        return False, "El subsistema ROS 2 no está corriendo."

    def publish_velocity(self, v: float, w: float) -> bool:
        if self._is_running and self.gateway_node:
            return self.gateway_node.publish_velocity(v, w)
        return False
    
    def check_connection(self) -> bool:
        if self._is_running and self.gateway_node:
            return self.gateway_node.check_connection_silently()
        return False
    
    # ==========================================
    # ¡NUEVO! API PARA EL VÍDEO
    # ==========================================
    def is_video_server_running(self) -> bool:
        if self._is_running and self.gateway_node:
            return self.gateway_node.check_video_server_silently()
        return False