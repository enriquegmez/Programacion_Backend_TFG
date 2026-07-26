## @file ros2_node_gateway.py
#  @brief Enlace principal entre el servidor asíncrono y el ecosistema de ROS 2.
#  @details Combina los gestores de Descubrimiento, Sensores, Acciones y Control
#           para realizar las peticiones del servidor.
#  @author Enrique Gómez
#  @date 2026

import time
import logging
import threading
from typing import Any, Optional, Callable, Dict, List, Tuple

import rclpy                                          # type: ignore[import]
from rclpy.node import Node                           # type: ignore[import]
from rclpy.subscription import Subscription           # type: ignore[import]
from rclpy.executors import SingleThreadedExecutor    # type: ignore[import]
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_sensor_data # type: ignore[import]

from std_msgs.msg import Bool, Float32, String        # type: ignore[import]
from sensor_msgs.msg import JointState, Image         # type: ignore[import]

from r2pilot_server.utils.constants import ControlEvent, SessionTimeout, RosMsgTypes
from r2pilot_server.ros.ros2_safety_node import R2PilotSafetyNode # type: ignore[import]

# --- Importación de los 4 Pilares (Managers de Dominio) ---
from r2pilot_server.ros.ros2_discovery import DiscoveryManager
from r2pilot_server.ros.ros2_sensor_manager import SensorManager
from r2pilot_server.ros.ros2_action_manager import ActionManager
from r2pilot_server.ros.ros2_control_manager import ControlManager


class R2PilotBridgeNode(Node):
    """!
    @brief Nodo central de ROS 2.
    @details Actúa como puente entre el servidor asíncrono y el ecosistema de ROS 2,
             combinando los 4 pilares de la arquitectura: Descubrimiento, Sensores, Acciones y Control.
    """
    def __init__(self) -> None:
        """!
        @brief Inicializa el nodo y los managers de dominio.
        @details Crea los publicadores y suscriptores base, y establece el estado inicial de las variables compartidas.
        """ 
        super().__init__('R2Pilot_bridge')
        self.logger = logging.getLogger("R2PilotBridgeNode")
        
        # Composición de Arquitectura (Los 4 Pilares)
        self.discovery = DiscoveryManager(self)
        self.sensors = SensorManager(self)
        self.actions = ActionManager(self)
        self.control = ControlManager(self)
        
        # Publicadores base de utilidades (Relay de vídeo genérico)
        self.rgb_relay_pub = self.create_publisher(Image, '/camera/rgb/relay', 10)

        # Estado Global del Robot Compartido
        self.latest_battery_pct: Optional[float] = None 
        self.latest_estop_active: Optional[bool] = False
        self.latest_is_charging: Optional[bool] = None
        self.current_joint_states: Dict[str, float] = {}
        self.safety_alert = "OK"
        self.is_connected = False

        # Suscripciones Críticas Base
        self.battery_sub: Optional[Subscription] = None
        self.estop_sub: Optional[Subscription] = None
        self.charging_sub: Optional[Subscription] = None
        self.camera_relay_sub: Optional[Subscription] = None

        
        
        
        # Suscripción pasiva al alertador del SafetyNode
        self.create_subscription(String, 'R2Pilot_teleop/safety_alert', self._safety_alert_callback, 10)
        
        # Monitorización continua de la posición de los motores
        self.create_subscription(JointState, '/joint_states', self._joint_states_callback, 10)
        
        # Descubrimiento del modelo URDF (Lectura única transitoria)
        qos_urdf = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self._urdf_callback, qos_urdf)

        # Timer de auto-descubrimiento para tópicos que aparecen dinámicamente
        self.discovery_timer = self.create_timer(SessionTimeout.AUTO_DISCOVERY_TOPICS_INTERVAL, self._discovery_timer_callback)
        self.logger.info("[BRIDGE] Puente ROS 2 iniciado. Composición de managers cargada.")

    # =========================================================================
    # CALLBACKS BÁSICOS DE ESTADO
    # =========================================================================

    def _joint_states_callback(self, msg: JointState) -> None:
        """! 
        @brief Actualiza el diccionario interno con la posición actual de cada articulación. 
        @details Este callback se suscribe al tópico /joint_states y mantiene un registro actualizado
                    de las posiciones de los motores del robot, que luego puede ser consultado por el servidor.
        @param msg Mensaje de tipo JointState recibido del tópico.
        """
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_states[name] = pos

    # Añade estos métodos junto a _joint_states_callback
    def _safety_alert_callback(self, msg: String) -> None:
        self.safety_alert = msg.data

    def _urdf_callback(self, msg: String) -> None:
        self.discovery.parse_urdf(msg.data)

    def _battery_callback(self, msg: Float32) -> None:
        self.latest_battery_pct = float(msg.data)

    def _estop_callback(self, msg: Bool) -> None:
        self.latest_estop_active = msg.data

    def _charging_callback(self, msg: Bool) -> None:
        self.latest_is_charging = msg.data

    def _discovery_timer_callback(self) -> None:
        """! 
        @brief Bucle de auto-descubrimiento recurrente.
        @details Escanea la red de ROS 2 buscando tópicos de energía, emergencia y cámaras 
                 para enlazar las suscripciones necesarias sobre la marcha.
        """
        # Si ya hemos encontrado la batería y el botón de emergencia, evitamos trabajo extra
        if self.battery_sub is not None and self.estop_sub is not None: return

        #Escaneo de tópicos activos en la red de ROS 2
        topics_and_types = self.get_topic_names_and_types()
        for name, types in topics_and_types:
            if self.battery_sub is None and '/power/battery_level' in name and RosMsgTypes.FLOAT32 in types:
               self.battery_sub = self.create_subscription(Float32, name, self._battery_callback, qos_profile_sensor_data)
                
            if self.estop_sub is None and '/power/is_emergency' in name:
                permisive_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
                self.estop_sub = self.create_subscription(Bool, name, self._estop_callback, permisive_qos)
                
            if self.charging_sub is None and '/power/is_charging' in name and RosMsgTypes.BOOL in types:
                self.charging_sub = self.create_subscription(Bool, name, self._charging_callback, qos_profile_sensor_data)
            
            if RosMsgTypes.JOINT_TRAJ in types:
                self.discovery.resolve_controller_joints_async(name)
            
            #if 'rgb' in name.lower() and 'image_raw' in name.lower():
             #       self.logger.info(f"[DISCOVERY] Enlazando relay de vídeo al tópico: {name}")
              #      self.camera_relay_sub = self.create_subscription(Image, name, self.rgb_relay_pub.publish, qos_profile_sensor_data)
    # =========================================================================
    # LÓGICA DE CONEXIÓN Y SUPERVISIÓN DE RED
    # =========================================================================

    def _get_fq_nodes(self) -> List[str]:
        """! 
        @brief Obtiene la lista de los nodos (con nombres completos) limpiando dobles barras. 
        @return Lista de nodos activos en la red de ROS 2, con nombres completos y sin duplicados.
        """
        return [f"{ns}/{name}".replace('//', '/') for name, ns in self.get_node_names_and_namespaces()]

    def connect(self) -> int:
        """!
        @brief Intenta establecer el vínculo lógico con un robot físico en la red DDS.
        @details Filtra daemons, herramientas de consola y nodos con bugs ('add_analyzer_node') 
                 para detectar la presencia real de la máquina y evitar falsos positivos.
        @return 0 si no hay robot, 1 si conexión es exitosa, 2 si hay conflicto de múltiples robots.
        """

        # Comprobación de nodos activos en la red de ROS 2, filtrando los que no son del robot real
        robot_nodes = [
            n for n in self._get_fq_nodes() 
            if n not in ['/R2Pilot_bridge', '/R2Pilot_safety_filter', '/web_video_server'] 
            and not n.startswith('/ros2cli') 
            and not n.startswith('/_ros2cli') 
            and not n.startswith('/launch') 
            and not n.startswith('/daemon')
            and 'add_analyzer_node' not in n  
        ]
        
        if len(robot_nodes) == 0: 
            return 0
            
        # Comprobación matemática de duplicados
        if len(robot_nodes) != len(set(robot_nodes)): 
            return 2
            
        self.is_connected = True
        self.logger.info("[CONEXIÓN] Robot ÚNICO detectado.")
        return 1

    def disconnect(self) -> None:
        """! @brief Rompe la conexión lógica de forma segura, parando acciones y motores. """
        if self.is_connected:
            self.logger.info("[DESCONEXIÓN] Aplicando freno de emergencia y apagando módulos.")
            self.actions.cancel_all()
            self.sensors.stop_all()
            self.control.stop_robot()
            self.is_connected = False
            self.control.is_control_active = False

    def check_connection_silently(self) -> int:
        """! 
        @brief Ejecuta la misma lógica que connect() pero sin alterar variables ni dejar logs. 
        @details Se utiliza para supervisar la conexión de forma periódica en el Watchdog.
        @return 0 si no hay robot, 1 si conexión es exitosa, 2 si hay conflicto de múltiples robots.
        """
        if not self.is_connected: 
            return 0
            
        robot_nodes = [
            n for n in self._get_fq_nodes() 
            if n not in ['/R2Pilot_bridge', '/R2Pilot_safety_filter', '/web_video_server'] 
            and not n.startswith('/ros2cli') 
            and not n.startswith('/_ros2cli') 
            and not n.startswith('/launch') 
            and not n.startswith('/daemon')
            and 'add_analyzer_node' not in n
        ]
        
        if len(robot_nodes) == 0: 
            return 0 
            
        # Comprobación matemática de duplicados
        if len(robot_nodes) != len(set(robot_nodes)): 
            return 2
            
        return 1
    
    def is_topic_active(self, topic_name: str) -> bool:
        """! 
        @brief Verifica si un topic específico está actualmente registrado en la red de ROS 2. 
        @param topic_name Nombre del tópico a verificar.
        @return True si el tópico está activo, False si no lo está.
        """
        clean = topic_name.strip()
        for name, _ in self.get_topic_names_and_types():
            if name == clean or name.endswith(f'/{clean.lstrip("/")}') or name == f'/{clean}': return True
        return False

    def check_video_server_silently(self) -> bool:
        """! 
        @brief Comprueba si el paquete web_video_server está corriendo en el ecosistema. 
        @details Se utiliza para determinar si el canal de vídeo puede ser abierto sin errores.
        @return True si web_video_server está activo, False si no lo está.
        """
        return '/web_video_server' in self._get_fq_nodes()


class Ros2Manager:
    """!
    @brief Controlador maestro del ciclo de vida del hilo de ROS 2 (Infraestructura).
    @details Expone el sub-ecosistema de ROS 2 al Director (director.py) de forma completamente abstracta y segura.
    """
    def __init__(self) -> None:
        """!
        @brief Inicializa el gestor de ROS 2 y prepara las variables de estado.
        """
        self.logger = logging.getLogger("Ros2Manager")
        self.gateway_node: Optional[R2PilotBridgeNode] = None
        self.safety_node: Optional[R2PilotSafetyNode] = None
        self.executor: Optional[SingleThreadedExecutor] = None
        self.spin_thread: Optional[threading.Thread] = None
        self._is_running = False

    # =========================================================================
    # CICLO DE VIDA (THREADING Y EXECUTOR)
    # =========================================================================

    def start(self) -> None:
        """! @brief Inicializa rclpy, instancia los nodos e inicia el hilo en segundo plano. """
        if self._is_running: return
        self.logger.info("[CORE] Arrancando motor ROS 2 (Gateway + Filtro)...")
        rclpy.init()
        self.gateway_node = R2PilotBridgeNode()
        self.safety_node = R2PilotSafetyNode()
        
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.gateway_node)
        self.executor.add_node(self.safety_node)
        
        self._is_running = True
        self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self.spin_thread.start()

    def _spin_loop(self) -> None:
        """! @brief Bucle infinito bloqueante que procesa los callbacks de ROS 2. """
        try:
            if self.executor: self.executor.spin()
        except Exception as e: 
            self.logger.error(f"Error crítico en rclpy executor: {e}")
        finally: 
            self.logger.info("[CORE] Hilo de ROS 2 finalizado.")

    def stop(self) -> None:
        """! @brief Apaga los actuadores físicos y destruye la memoria de ROS 2 de forma limpia. """
        if not self._is_running: return
        self._is_running = False
        try:
            if self.gateway_node: self.gateway_node.disconnect()
            if self.safety_node: self.safety_node.stop_robot()
            time.sleep(0.2)
        except Exception: pass
        
        try:
            if self.executor: self.executor.shutdown()
            if self.gateway_node: self.gateway_node.destroy_node()
            if self.safety_node: self.safety_node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
            if self.spin_thread: self.spin_thread.join(timeout=2.0)
        except Exception: pass

    # =========================================================================
    # BIBLIOTECA DE FUNCIONES EXPUESTA AL DIRECTOR (director.py)
    # =========================================================================
    
    # --- Gestión de Sesión y Conexión ---
    def connect_to_robot(self) -> int:
        return self.gateway_node.connect() if self._is_running and self.gateway_node else 0

    def disconnect_from_robot(self) -> None:
        if self._is_running and self.gateway_node: self.gateway_node.disconnect()

    def check_connection(self) -> int:
        return self.gateway_node.check_connection_silently() if self._is_running and self.gateway_node else 0
        
    # --- Control Físico ---
    def stop_robot(self) -> None:
        if self._is_running and self.gateway_node: self.gateway_node.control.stop_robot()

    def set_control_mode(self, event: str, control_type: str = "TELEOP", topic: str = "cmd_vel") -> Tuple[bool, str]:
        if self._is_running and self.gateway_node and self.safety_node:
            success, msg = self.gateway_node.control.set_control_mode(event, control_type, topic)
            if success and event == ControlEvent.START:
                if control_type != "JOINT": self.safety_node.set_target_topic(msg)
            return success, msg
        return False, "ROS 2 no está corriendo."

    def publish_velocity(self, v: float, w: float) -> Tuple[bool, str]:
        return self.gateway_node.control.publish_velocity(v, w) if self._is_running and self.gateway_node else (False, "Apagado.")
    
    def publish_joint_position(self, joint_name: str, value: float) -> bool:
        return self.gateway_node.control.publish_joint_position(joint_name, value) if self._is_running and self.gateway_node else False

    # --- Acciones Prologadas (Action Servers) ---
    def set_action_feedback_callback(self, callback: Callable[[bool, bool, int, str], None]) -> None:
        if self._is_running and self.gateway_node: self.gateway_node.actions.set_feedback_callback(callback)

    def get_available_actions(self) -> Tuple[bool, Any]:
        return self.gateway_node.actions.get_available_actions() if self._is_running and self.gateway_node else (False, "Apagado.")

    def execute_action(self, action_type: str, target: str) -> Tuple[bool, str]:
        return self.gateway_node.actions.execute_action(target) if self._is_running and self.gateway_node else (False, "Apagado.")

    def stop_action(self, action_type: str, target: str) -> bool:
        return self.gateway_node.actions.stop_action(target) if self._is_running and self.gateway_node else False

    # --- Telemetría y Sensores ---
    def start_sensor_stream(self, topic: str, callback: Callable) -> bool:
        return self.gateway_node.sensors.start_stream(topic, callback) if self._is_running and self.gateway_node else False
        
    def stop_sensor_stream(self, topic: str) -> None:
        if self._is_running and self.gateway_node: self.gateway_node.sensors.stop_stream(topic)

    def get_available_sensors(self) -> List[Dict[str, str]]:
        return self.gateway_node.discovery.get_available_sensors() if self._is_running and self.gateway_node else []

    # --- Vídeo y Capacidades ---
    def is_video_server_running(self) -> bool:
        return self.gateway_node.check_video_server_silently() if self._is_running and self.gateway_node else False
    
    def is_topic_active(self, topic_name: str) -> bool:
        return self.gateway_node.is_topic_active(topic_name) if self._is_running and self.gateway_node else False
    
    def get_teleop_topics(self) -> List[str]:
        return self.gateway_node.discovery.get_teleop_topics() if self._is_running and self.gateway_node else []

    def get_camera_topics(self) -> List[str]:
        return self.gateway_node.discovery.get_camera_topics() if self._is_running and self.gateway_node else []

    def get_robot_capabilities(self) -> Dict[str, Any]:
        if self._is_running and self.gateway_node:
            return self.gateway_node.discovery.build_robot_info(
                self.gateway_node.latest_battery_pct, 
                self.gateway_node.latest_estop_active, 
                self.gateway_node.latest_is_charging,
                self.gateway_node.current_joint_states
            )
        return {}

    # --- Inspección Global (Discovery) ---
    def get_all_topics(self) -> Dict[str, List[str]]:
        return self.gateway_node.discovery.get_all_topics() if self._is_running and self.gateway_node else {}

    def get_all_services(self) -> Dict[str, List[str]]:
        return self.gateway_node.discovery.get_all_services() if self._is_running and self.gateway_node else {}

    def get_all_actions(self) -> Dict[str, List[str]]:
        return self.gateway_node.discovery.get_all_actions() if self._is_running and self.gateway_node else {}