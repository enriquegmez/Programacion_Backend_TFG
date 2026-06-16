"""
ros_node_handler.py
Enlace entre el servidor asíncrono y el ecosistema de ROS 2.
Maneja el ciclo de vida de rclpy en un hilo dedicado y publica en /cmd_vel.
"""

import logging
import threading
import rclpy  # type: ignore[import]
import time
import os       # ¡NUEVO! Para leer variables de entorno como ROS_DOMAIN_ID
import socket   # ¡NUEVO! Para obtener el nombre del host
from rclpy.node import Dict, Node # type: ignore[import]
from geometry_msgs.msg import Twist # type: ignore[import]
from rclpy.executors import SingleThreadedExecutor # type: ignore[import]
from sensor_msgs.msg import BatteryState # type: ignore[import]
from std_msgs.msg import Bool # type: ignore[import]

from app_tiago.utils.constants import ControlEvent, RosMsgTypes, RobotInfoKeys, DiscoveryConfig
from app_tiago.ros.ros2_core_node import SafetyFilterNode

class TiagoBridgeNode(Node):
    def __init__(self):
        super().__init__('app_tiago_bridge')
        self.logger = logging.getLogger("TiagoBridgeNode")
        
        # Publicador real al tópico de velocidad de Tiago
        self.vel_publisher = self.create_publisher(msg_type=Twist, 
                                                   topic='web_teleop/cmd_vel_raw', 
                                                   qos_profile=10)
        
        # ==========================================
        # ¡NUEVO! VARIABLES DE CACHÉ PARA ESTADO VITAL
        # ==========================================
        self.latest_battery_pct = 100.0
        self.latest_estop_active = False

        # Variables para guardar las suscripciones dinámicas
        self.battery_sub = None
        self.estop_sub = None

        # ==========================================
        # SOLUCIÓN C: Temporizador de Auto-Descubrimiento DDS
        # ==========================================
        # Se ejecuta cada 2.0 segundos para buscar topics críticos
        self.discovery_timer = self.create_timer(2.0, self._discovery_timer_callback)

        self.logger.info("Puente ROS 2 iniciado. Publicando raw en: /web_teleop/cmd_vel_raw")

        # Estado de seguridad
        self.is_connected = False
        self.is_control_active = False

    def _discovery_timer_callback(self):
        """Busca topics de Batería y E-Stop periódicamente hasta encontrarlos."""
        # Si ya hemos encontrado ambos, no hacemos más polling para ahorrar CPU
        if self.battery_sub is not None and self.estop_sub is not None:
            return

        topics_and_types = self.get_topic_names_and_types()

        for name, types in topics_and_types:
            # 1. Buscar Batería (SOLUCIÓN 1)
            if self.battery_sub is None and 'sensor_msgs/msg/BatteryState' in types:
                self.logger.info(f"¡Topic de Batería auto-descubierto en: {name}!")
                self.battery_sub = self.create_subscription(BatteryState, name, self._battery_callback, 10)
                
            # 2. Buscar E-Stop (SOLUCIÓN B)
            if self.estop_sub is None and 'std_msgs/msg/Bool' in types:
                # Filtrar nombres comunes de parada de emergencia
                if 'estop' in name.lower() or 'emergency' in name.lower():
                    self.logger.info(f"¡Topic de E-Stop auto-descubierto en: {name}!")
                    self.estop_sub = self.create_subscription(Bool, name, self._estop_callback, 10)
    
    # Callbacks asíncronos para actualizar la caché
    def _battery_callback(self, msg: BatteryState):
        if msg.percentage <= 1.0:
            self.latest_battery_pct = float(msg.percentage * 100.0)
        else:
            self.latest_battery_pct = float(msg.percentage)

    def _estop_callback(self, msg: Bool):
        # Asumimos que True = Botón pulsado (Emergencia)
        self.latest_estop_active = msg.data

    def connect(self) -> bool:
        """Verifica que exista EXACTAMENTE UN robot conectado a la red ROS 2."""
        nodos_activos = self.get_node_names()
        nodos_nuestros = ['app_tiago_bridge', 'app_safety_filter', 'web_video_server']
        
        nodos_robot = [
            nodo for nodo in nodos_activos 
            if nodo not in nodos_nuestros 
            and not nodo.startswith('ros2cli') 
            and not nodo.startswith('_ros2cli') 
            and not nodo.startswith('launch') 
            and not nodo.startswith('daemon')
        ]
        
        # 1. ¿No hay robot?
        if len(nodos_robot) == 0:
            self.logger.warning("Conexión rechazada: No se ha detectado el robot.")
            return False
            
        # 2. ¿Hay MÚLTIPLES robots? (Comprobamos si hay nombres de nodos repetidos)
        if len(nodos_robot) != len(set(nodos_robot)):
            self.logger.error("Conexión rechazada: Múltiples robots detectados (Choque de nodos en red).")
            return False
            
        # 3. ¿Hay MÚLTIPLES simuladores? (Contamos los nodos principales)
        #cerebros = [n for n in nodos_robot if 'gazebo' in n or 'robot_state_publisher' in n]
        #if len(cerebros) > 1:
        #    self.logger.error("Conexión rechazada: Múltiples instancias de Gazebo o Tiago detectadas.")
        #    return False

        self.is_connected = True
        self.logger.info("Robot ÚNICO detectado. Conexión segura establecida.")
        return True

    def disconnect(self):
        """Frena el robot y rompe la conexión lógica."""
        if self.is_connected:
            self.logger.info("Desconectando del Tiago. Aplicando freno de emergencia.")
            self.stop_robot()
            self.is_connected = False
            self.is_control_active = False

    def check_connection_silently(self) -> int:
        """
        Vigila la red silenciosamente. 
        Retorna: 1 (OK), 0 (Robot Desconectado), 2 (Múltiples Robots / Conflicto)
        """
        if not self.is_connected:
            return 0
            
        nodos_activos = self.get_node_names()
        nodos_nuestros = ['app_tiago_bridge', 'app_safety_filter', 'web_video_server']
        
        nodos_robot = [
            nodo for nodo in nodos_activos 
            if nodo not in nodos_nuestros 
            and not nodo.startswith('ros2cli') 
            and not nodo.startswith('_ros2cli') 
            and not nodo.startswith('launch') 
            and not nodo.startswith('daemon')
        ]
        
        if len(nodos_robot) == 0:
            return 0 # Se ha apagado
            
        # Si de repente aparecen duplicados, alguien ha encendido otro robot
        if len(nodos_robot) != len(set(nodos_robot)):
            return 2 # Conflicto
            
        cerebros = [n for n in nodos_robot if 'gazebo' in n or 'robot_state_publisher' in n]
        if len(cerebros) > 1:
            return 2 # Conflicto
            
        return 1 # Todo correcto
    
    def is_topic_active(self, topic_name: str) -> bool:
        """Comprueba en la red ROS 2 si un topic existe, sin importar su tipo de mensaje."""
        clean_topic = topic_name.strip()
        topics_and_types = self.get_topic_names_and_types()
        
        for name, types in topics_and_types:
            if name == clean_topic or name.endswith(f'/{clean_topic.lstrip("/")}') or name == f'/{clean_topic}':
                return True
                
        return False

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


    def get_teleop_topics(self) -> list[str]:
        """Devuelve topics para teleoperación de base móvil ordenados por relevancia.

        Selecciona tópicos Twist/TwistStamped que claramente son para control de
        base móvil (requieren palabras clave conocidas como cmd_vel, teleop, etc).
        Rechaza topics Twist desconocidos o sospechosos por seguridad."""
        topics_and_types = self.get_topic_names_and_types()
        safe_teleop_topics: list[str] = []

        # Palabras clave SEGURAS: indican que es control de base móvil
        safe_base_keywords = [
            'cmd_vel',
            'cmd_vel_unstamped',
            'cmd_vel_stamped',
            'teleop',
            'velocity',
            'twist',
            'diff_drive',
            'base_controller',
            'mobile_base_controller',
            'wheel',
            'drive',
            'movement'
        ]

        def _topic_priority(topic_name: str) -> int:
            lower_name = topic_name.lower()
            for index, keyword in enumerate(safe_base_keywords):
                if keyword in lower_name:
                    return index
            return len(safe_base_keywords)

        for name, types in topics_and_types:
            if RosMsgTypes.TWIST in types or RosMsgTypes.TWIST_STAMPED in types:
                lower_name = name.lower()
                
                # SEGURIDAD: Solo incluir si tiene palabra clave CONOCIDA de base
                # Rechaza topics Twist desconocidos o inesperados
                if any(safe_kw in lower_name for safe_kw in safe_base_keywords):
                    safe_teleop_topics.append(name)
                else:
                    self.logger.debug(
                        f"Topic Twist excluido (no reconocido como base móvil): {name}. "
                        f"Debe contener: {', '.join(safe_base_keywords[:3])}..."
                    )

        # Ordenar por prioridad semántica
        safe_teleop_topics.sort(key=lambda topic: (_topic_priority(topic), len(topic), topic))
        
        if not safe_teleop_topics:
            self.logger.warning(
                "No se encontraron topics seguros de teleoperación. "
                "Verifica que el robot tenga un topic cmd_vel válido."
            )
        
        return safe_teleop_topics



    def get_camera_topics(self) -> list[str]:
        """Devuelve tópicos de cámara ordenados por relevancia para web_video_server.

        Detecta topics de tipo Image o CompressedImage, filtra ruido (depth, masks, etc.)
        y prioriza flujos RGB típicos de cámaras reales (`camera`, `image_raw`, `rgb`)."""
        topics_and_types = self.get_topic_names_and_types()
        camera_topics: list[str] = []

        # Palabras clave a excluir (imágenes procesadas o no-RGB)
        exclude_keywords = [
            'depth',
            'disparity',
            'mask',
            'segmentation',
            'semantic',
            'instance',
            'optical_flow',
            'stereo'
        ]

        # Palabras clave para priorizar (cámaras RGB reales)
        priority_keywords = [
            'camera',
            'image_raw',
            'rgb',
            'color',
            'front',
            'main'
        ]

        def _camera_priority(topic_name: str) -> int:
            lower_name = topic_name.lower()
            for index, keyword in enumerate(priority_keywords):
                if keyword in lower_name:
                    return index
            return len(priority_keywords)

        for topic_name, types in topics_and_types:
            if RosMsgTypes.IMAGE in types or RosMsgTypes.COMPRESSED_IMAGE in types:
                lower_name = topic_name.lower()
                
                # Excluir imágenes que no son cámaras RGB
                if any(excl in lower_name for excl in exclude_keywords):
                    continue
                
                camera_topics.append(topic_name)

        # Ordenar: primero por relevancia semántica, luego por longitud, luego lexicográficamente
        camera_topics.sort(key=lambda name: (_camera_priority(name), len(name), name))
        return camera_topics
    
    # ==========================================
    # EL NUEVO DETECTIVE ULTRA-UNIVERSAL
    # ==========================================
    def get_robot_info(self) -> dict:
        """Radiografía universal del robot aplicando heurísticas avanzadas.
        
        Detecta capacidades hardware por análisis de topics y servicios ROS 2.
        Usa `if` independientes (no `elif`) para detectar múltiples capacidades."""
        hostname = socket.gethostname()
        domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
        
        battery_pct = self.latest_battery_pct
        e_stop_active = self.latest_estop_active
        
        has_base = False
        cameras_list: list[dict[str, str]] = []  
        has_manipulator = False
        has_gripper = False
        has_lidar = False
        has_imu = False
        has_odom = False
        has_nav = False
        has_moveit = False

        detected_camera_roots = set()
        topics_and_types = self.get_topic_names_and_types()

        for topic_name, types in topics_and_types:
            topic_lower = topic_name.lower()
            
            # --- Base Móvil (Twist/TwistStamped) ---
            # Con `if` para detectar independientemente
            if RosMsgTypes.TWIST in types or RosMsgTypes.TWIST_STAMPED in types:
                if any(keyword in topic_lower for keyword in DiscoveryConfig.BASE_KEYWORDS):
                    has_base = True
            
            # --- Cámaras RGB (CameraInfo) ---
            if RosMsgTypes.CAMERA_INFO in types:
                hw_root = topic_name
                for suffix in DiscoveryConfig.CAMERA_CLEANUP_SUFFIXES:
                    hw_root = hw_root.replace(suffix, "")
                
                if hw_root not in detected_camera_roots:
                    detected_camera_roots.add(hw_root)
                    display_name = hw_root.split('/')[-1].replace('_', ' ').title()
                    if not display_name: 
                        display_name = DiscoveryConfig.DEFAULT_CAMERA_NAME

                    cameras_list.append({"name": display_name})
            
            # --- LiDAR (LaserScan o PointCloud2) ---
            if RosMsgTypes.LASER_SCAN in types:
                has_lidar = True
            elif RosMsgTypes.POINT_CLOUD2 in types:
                # Evitar false positives de software de visión (depth clouds)
                if not any(excl in topic_lower for excl in DiscoveryConfig.LIDAR_EXCLUDE_KEYWORDS):
                    has_lidar = True
            
            # --- Brazos / Manipuladores ---
            # JointTrajectory es la forma estándar de mover brazos
            if RosMsgTypes.JOINT_TRAJ in types:
                if any(keyword in topic_lower for keyword in DiscoveryConfig.ARM_KEYWORDS):
                    has_manipulator = True
            
            # --- IMU (Sensor inercial) ---
            if RosMsgTypes.IMU in types:
                has_imu = True
            
            # --- Odometría ---
            if RosMsgTypes.ODOMETRY in types:
                has_odom = True
            
            # --- Gripper / Actuadores finales ---
            if RosMsgTypes.JOINT_STATE in types or 'std_msgs/msg/Float64' in types:
                if any(keyword in topic_lower for keyword in ['gripper', 'hand', 'actuator']):
                    has_gripper = True
            
            # --- Nav2 (Mapas de navegación) ---
            if RosMsgTypes.OCCUPANCY_GRID in types:
                has_nav = True
            
            # --- MoveIt (Planificación de movimiento) ---
            if RosMsgTypes.MOVEIT_PLANNING_SCENE in [t.lower() for t in types]:
                has_moveit = True

        diagnostic = (
            f"Base={has_base}, Cámaras={len(cameras_list)}, "
            f"Brazo={has_manipulator}, Gripper={has_gripper}, "
            f"LiDAR={has_lidar}, IMU={has_imu}, Odom={has_odom}, "
            f"Nav2={has_nav}, MoveIt={has_moveit}"
        )
        self.logger.info(f"Escaneo universal: {diagnostic}")
        
        return {
            RobotInfoKeys.IDENTITY: {
                "hostname": hostname,
                "domain_id": domain_id,
            },
            RobotInfoKeys.STATUS: {
                "battery_pct": battery_pct,
                "e_stop_active": e_stop_active
            },
            RobotInfoKeys.CAPABILITIES: {
                RobotInfoKeys.HAS_BASE: has_base,
                RobotInfoKeys.CAMERAS: cameras_list,
                RobotInfoKeys.HAS_MANIPULATOR: has_manipulator,
                RobotInfoKeys.HAS_GRIPPER: has_gripper,
                RobotInfoKeys.HAS_IMU: has_imu,
                RobotInfoKeys.HAS_ODOMETRY: has_odom,
                RobotInfoKeys.HAS_LIDAR: has_lidar,
                RobotInfoKeys.HAS_NAV: has_nav,
                RobotInfoKeys.HAS_MOVEIT: has_moveit
            }
        }


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
    
    def check_connection(self) -> int:
        # ¡IMPORTANTE! Cambiamos -> bool por -> int para gestionar los 3 estados
        if self._is_running and self.gateway_node:
            return self.gateway_node.check_connection_silently()
        return 0
    
    # ==========================================
    # ¡NUEVO! API PARA EL VÍDEO
    # ==========================================
    def is_video_server_running(self) -> bool:
        if self._is_running and self.gateway_node:
            return self.gateway_node.check_video_server_silently()
        return False
    
    def is_topic_active(self, topic_name: str) -> bool:
        """Expone la comprobación de topics genéricos para el Router."""
        if self._is_running and self.gateway_node:
            return self.gateway_node.is_topic_active(topic_name)
        return False
    
    def get_teleop_topics(self) -> list[str]:
        if self._is_running and self.gateway_node:
            return self.gateway_node.get_teleop_topics()
        return []

    def get_camera_topics(self) -> list[dict[str, str]]:
        if self._is_running and self.gateway_node:
            return self.gateway_node.get_camera_topics()
        return []
    
    # ==========================================
    # ¡NUEVO! API PARA EL DICCIONARIO
    # ==========================================
    def get_robot_capabilities(self) -> dict:
        """Expone la información del robot para responder a consultas (QueryResp)."""
        if self._is_running and self.gateway_node:
            return self.gateway_node.get_robot_info()
        return {}