"""
ros_node_handler.py
Enlace entre el servidor asíncrono y el ecosistema de ROS 2.
Maneja el ciclo de vida de rclpy en un hilo dedicado y publica en /cmd_vel.
"""

import logging
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from app_tiago.utils.constants import ControlEvent

class TiagoBridgeNode(Node):
    def __init__(self):
        super().__init__('app_tiago_bridge')
        self.logger = logging.getLogger("TiagoBridgeNode")
        
        # Publicador real al tópico de velocidad de Tiago
        self.vel_publisher = self.create_publisher(msg_type=Twist, 
                                                   topic='/cmd_vel', 
                                                   qos_profile=10)
        self.logger.info("Publicador ROS 2 iniciado en el tópico: /cmd_vel")

        # Estado de seguridad
        self.is_connected = False
        self.is_control_active = False

    def connect(self) -> bool:
        """Establece la conexión lógica con el robot."""
        # Comprobación real de ROS 2: Verificamos si hay alguien escuchando /cmd_vel
        # En el simulador (Gazebo/Webots) o robot real, debería haber al menos 1 suscriptor
        subs_count = self.vel_publisher.get_subscription_count()
        if subs_count == 0:
            self.logger.warning("Conexión rechazada: No se detecta ningún nodo suscrito a /cmd_vel (¿Simulador apagado?).")
            # Nota: Si tu simulador usa un topic diferente a /cmd_vel (ej. /nav_vel o /mobile_base_controller/cmd_vel_unstamped),
            # deberás cambiarlo arriba en el create_publisher. Si te bloquea las pruebas, pon un 'return True' aquí temporalmente.
            return True  #PONER EN FALSE CUANDO SE SEPA EL TOPIC
            
        self.is_connected = True
        self.logger.info(f"Conectado al Tiago. Suscriptores detectados en /cmd_vel: {subs_count}")
        return True

    def disconnect(self):
        """Frena el robot y rompe la conexión lógica."""
        if self.is_connected:
            self.logger.info("Desconectando del Tiago. Aplicando freno de emergencia.")
            self.stop_robot()
            self.is_connected = False
            self.is_control_active = False

    def set_control_mode(self, event: str) -> bool:
        """Habilita o deshabilita el movimiento desde el joystick."""
        if not self.is_connected:
            self.logger.error("Cambio de modo denegado: El robot no está conectado.")
            return False

        if event == ControlEvent.START:
            self.is_control_active = True
            self.logger.info("Control de Joystick ACTIVADO.")
            return True
            
        elif event == ControlEvent.STOP:
            self.is_control_active = False
            self.logger.info("Control de Joystick DESACTIVADO. Frenando robot.")
            self.stop_robot()
            return True
            
        return False

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
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.vel_publisher.publish(msg)


class Ros2Manager:
    """
    Controlador del hilo de ROS 2. Aísla rclpy.spin() del bucle asyncio del servidor web.
    """
    def __init__(self):
        self.logger = logging.getLogger("Ros2Manager")
        self.node = None
        self.spin_thread = None
        self._is_running = False

    def start(self):
        """Inicializa el ecosistema ROS 2 y su hilo de ejecución."""
        if self._is_running:
            return

        self.logger.info("Arrancando subsistema ROS 2...")
        rclpy.init()
        self.node = TiagoBridgeNode()
        self._is_running = True

        self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self.spin_thread.start()

    def _spin_loop(self):
        """Mantiene vivo el nodo para que pueda enviar/recibir mensajes continuamente."""
        try:
            rclpy.spin(self.node)
        except Exception as e:
            self.logger.error(f"Error crítico en rclpy.spin(): {e}")
        finally:
            self.logger.info("Hilo de ROS 2 finalizado.")

    def stop(self):
        """Apaga ROS 2 de forma limpia garantizando que el robot se detenga."""
        if not self._is_running:
            return

        self.logger.info("Apagando subsistema ROS 2...")
        self._is_running = False
        
        if self.node:
            self.node.disconnect()  # Freno garantizado antes de morir
            self.node.destroy_node()
            
        if rclpy.ok():
            rclpy.shutdown()
            
        if self.spin_thread:
            self.spin_thread.join(timeout=2.0)

    # ==========================================
    # API PÚBLICA PARA EL ROUTER
    # ==========================================
    def connect_to_robot(self) -> bool:
        if self._is_running and self.node:
            return self.node.connect()
        return False

    def disconnect_from_robot(self):
        if self._is_running and self.node:
            self.node.disconnect()

    def set_control_mode(self, event: str) -> bool:
        if self._is_running and self.node:
            return self.node.set_control_mode(event)
        return False

    def publish_velocity(self, v: float, w: float) -> bool:
        if self._is_running and self.node:
            return self.node.publish_velocity(v, w)
        return False