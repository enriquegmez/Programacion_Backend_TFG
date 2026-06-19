"""
ros_node_handler.py
Enlace entre el servidor asíncrono y el ecosistema de ROS 2.
Maneja el ciclo de vida de rclpy en un hilo dedicado y publica en /cmd_vel.
"""

import logging
import threading
import rclpy # type: ignore[import]
import time
import os
import socket
from typing import Any, Optional, Callable # <-- ¡NUEVO! Importamos Callable
from rclpy.node import Node  # type: ignore[import]
from geometry_msgs.msg import Twist # type: ignore[import]
from rclpy.executors import SingleThreadedExecutor # type: ignore[import]
from rclpy.action import ActionClient, CancelResponse # type: ignore[import]
from sensor_msgs.msg import BatteryState # type: ignore[import]
from std_msgs.msg import Bool # type: ignore[import]
from rclpy.client import Client
from rclpy.subscription import Subscription

from play_motion2_msgs.action import PlayMotion2 # type: ignore[import]
from play_motion2_msgs.srv import GetMotionInfo, ListMotions # type: ignore[import]

from app_tiago.utils.constants import ControlEvent, RosMsgTypes, RobotInfoKeys, DiscoveryConfig
from app_tiago.ros.ros2_core_node import SafetyFilterNode

class TiagoBridgeNode(Node):
    def __init__(self) -> None: 
        super().__init__('app_tiago_bridge')
        self.logger = logging.getLogger("TiagoBridgeNode")
        
        # Publicador real al tópico de velocidad de Tiago
        self.vel_publisher = self.create_publisher(msg_type=Twist, 
                                                   topic='web_teleop/cmd_vel_raw', 
                                                   qos_profile=10)
        
        self.latest_battery_pct = 100.0
        self.latest_estop_active = False
        self.latest_is_charging= False

        self.battery_sub: Optional[Subscription] = None
        self.estop_sub: Optional[Subscription] = None
        self.charging_sub: Optional[Subscription] = None

        # --- Clientes ROS para Acciones (PlayMotion) ---
        self.play_motion_action_client: Optional[ActionClient] = None
        self.list_motions_client: Optional[Client] = None
        self.get_motion_info_client: Optional[Client] = None
        self.play_motion_action_name: Optional[str] = None
        self.play_motion_list_motions_service_name: Optional[str] = None
        self.play_motion_get_motion_info_service_name: Optional[str] = None
        self.play_motion_initialized = False

        self.current_goal_handle = None
        self.current_action_name: Optional[str] = None
        self._action_lock = threading.Lock()
        
        # ¡NUEVO! Referencia a la función del Router que enviará el WebSocket
        self._router_feedback_callback: Optional[Callable[[bool, int, str], None]] = None

        self.discovery_timer = self.create_timer(2.0, self._discovery_timer_callback)
        self.logger.info("Puente ROS 2 iniciado. Publicando raw en: /web_teleop/cmd_vel_raw")

        self.is_connected = False
        self.is_control_active = False

         # --- INICIO PRUEBA ---

        #self.latest_battery_pct = 100.0

        #self.latest_is_charging = False

        #self.latest_has_ft_sensor = False

        #self.latest_has_play_motion = False

        #self.latest_has_base = True

        #self.test_start_time = time.time()

        #self.test_timer = self.create_timer(5.0, self._mock_test_callback)

        #--- FIN PRUEBA --- 

        # --- INICIO PRUEBA ---

    #def _mock_test_callback(self):

        # 1. Bajar 1% cada 5 segundos

        # if self.latest_battery_pct > 0:

            # self.latest_battery_pct -= 1.0

        # 2. Comprobar si ha pasado 1 minuto (60 segundos)

        # tiempo_pasado = time.time() - self.test_start_time

#       if tiempo_pasado >= 20.0:

#           self.latest_is_charging = True

            # # Aquí forzamos el cambio de capacidades tras el minuto

            # # Necesitamos pasárselo a get_robot_info

            # self.latest_has_ft_sensor = True

            # self.latest_has_play_motion = True

            # self.latest_has_base = False

        # else:

            # self.latest_has_ft_sensor = False

            # self.latest_has_play_motion = False

            # self.latest_has_base = True

        # --- FIN PRUEBA --- 

    def _discovery_timer_callback(self) -> None:
        if self.battery_sub is not None and self.estop_sub is not None:
            return

        topics_and_types = self.get_topic_names_and_types()

        for name, types in topics_and_types:
            if self.battery_sub is None and 'sensor_msgs/msg/BatteryState' in types:
                self.logger.info(f"¡Topic de Batería auto-descubierto en: {name}!")
                self.battery_sub = self.create_subscription(BatteryState, name, self._battery_callback, 10)
                
            if self.estop_sub is None and 'std_msgs/msg/Bool' in types:
                if 'estop' in name.lower() or 'emergency' in name.lower():
                    self.logger.info(f"¡Topic de E-Stop auto-descubierto en: {name}!")
                    self.estop_sub = self.create_subscription(Bool, name, self._estop_callback, 10)

            if self.charging_sub is None and 'std_msgs/msg/Bool' in types:
                if 'charge' in name.lower() or 'charging' in name.lower():
                    self.logger.info(f"¡Topic de Carga auto-descubierto en: {name}!")
                    self.charging_sub = self.create_subscription(Bool, name, self._charging_callback, 10)
    
    def _battery_callback(self, msg: BatteryState):
        if msg.percentage <= 1.0:
            self.latest_battery_pct = float(msg.percentage * 100.0)
        else:
            self.latest_battery_pct = float(msg.percentage)

    def _charging_callback(self, msg: Bool):
        self.latest_is_charging = msg.data

    def _estop_callback(self, msg: Bool):
        self.latest_estop_active = msg.data

    def connect(self) -> int:
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
            self.logger.warning("Conexión rechazada: No se ha detectado el robot.")
            return 0
            
        nodos_sin_bugs = [n for n in nodos_robot if 'add_analyzer_node' not in n]
        if len(nodos_sin_bugs) != len(set(nodos_sin_bugs)):
            self.logger.error("Conexión rechazada: Múltiples robots detectados (Choque de nodos en red).")
            return 2
            
        self.is_connected = True
        self.logger.info("Robot ÚNICO detectado. Conexión segura establecida.")
        return 1

    def disconnect(self):
        if self.is_connected:
            self.logger.info("Desconectando del Tiago. Aplicando freno de emergencia.")
            self.stop_robot()
            self.is_connected = False
            self.is_control_active = False

    def check_connection_silently(self) -> int:
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
            return 0 
            
        nodos_sin_bugs = [n for n in nodos_robot if 'add_analyzer_node' not in n]
        if len(nodos_sin_bugs) != len(set(nodos_sin_bugs)):
            self.logger.error("Conexión rechazada: Múltiples robots detectados (Choque de nodos en red).")
            return 2
            
        return 1 
    
    def is_topic_active(self, topic_name: str) -> bool:
        clean_topic = topic_name.strip()
        topics_and_types = self.get_topic_names_and_types()
        
        for name, types in topics_and_types:
            if name == clean_topic or name.endswith(f'/{clean_topic.lstrip("/")}') or name == f'/{clean_topic}':
                return True
        return False

    def check_video_server_silently(self) -> bool:
        try:
            nodos_activos = self.get_node_names()
            return 'web_video_server' in nodos_activos
        except Exception as e:
            self.logger.error(f"Error buscando el servidor de vídeo: {e}")
            return False
        
    def validate_topic(self, topic_name: str) -> tuple[bool, str]:
        clean_topic = topic_name.strip()
        topics_and_types = self.get_topic_names_and_types()
        
        for name, types in topics_and_types:
            if name == clean_topic or name.endswith(f'/{clean_topic.lstrip("/")}') or name == f'/{clean_topic}':
                if 'geometry_msgs/msg/Twist' in types:
                    return True, name
                else:
                    return False, f"El topic '{name}' existe, pero no acepta Twist. Usa: {types[0]}"
        return False, f"El topic '{clean_topic}' no se ha encontrado."
    
    def set_control_mode(self, event: str, topic: str = "cmd_vel") -> tuple[bool, str]:
        if not self.is_connected:
            return False, "Robot no conectado lógicamente."

        if event == ControlEvent.START:
            is_valid, msg = self.validate_topic(topic)
            if not is_valid:
                return False, msg
            self.is_control_active = True
            return True, msg
            
        elif event == ControlEvent.STOP:
            self.is_control_active = False
            self.stop_robot()
            return True, ""
            
        return False, "Evento desconocido."

    def publish_velocity(self, v: float, w: float) -> bool:
        if not self.is_connected or not self.is_control_active:
            return False

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        
        self.vel_publisher.publish(msg)
        return True

    def stop_robot(self):
        try:
            if rclpy.ok() and self.vel_publisher is not None:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.vel_publisher.publish(msg)
        except Exception as e:
            self.logger.debug(f"Freno omitido: {e}")

    # ==========================================
    # LÓGICA DE ACTIONS Y PLAY MOTION
    # ==========================================
    def _wait_for_action_server(self, timeout_sec: float = 5.0) -> bool:
        if self.play_motion_action_client is None:
            return False
        if self.play_motion_action_client.server_is_ready():
            return True
        return self.play_motion_action_client.wait_for_server(timeout_sec=timeout_sec)

    def _wait_for_service(self, client: Any, timeout_sec: float = 5.0) -> bool:
        if client.service_is_ready():
            return True
        return client.wait_for_service(timeout_sec=timeout_sec)

    def _wait_for_future(self, future: Any, timeout_sec: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done()

    def _discover_play_motion_endpoints(self) -> bool:
        if self.play_motion_initialized:
            return True

        services = self.get_service_names_and_types()
        service_names = [name for name, _ in services]

        # --- DEBUG TEMPORAL ---
        self.logger.info(f"Servicios detectados en red: {service_names}")
        # ----------------------

        candidates = [name for name in service_names if 'play_motion' in name and name.endswith('/list_motions')]

        self.logger.info(f"Candidatos encontrados: {candidates}") # DEBUG

        if not candidates:
            return False

        # --- ¡SOLUCIÓN AL ERROR DE RSPLIT! ---
        # Guardamos en una variable local para que el linter sepa 100% que es un string
        found_service = candidates[0]
        self.play_motion_list_motions_service_name = found_service
        
        # Hacemos el rsplit sobre la variable local
        base = found_service.rsplit('/', 1)[0]
        
        self.play_motion_action_name = base
        self.play_motion_get_motion_info_service_name = f"{base}/get_motion_info"

        self.play_motion_action_client = ActionClient(self, PlayMotion2, self.play_motion_action_name)
        self.list_motions_client = self.create_client(ListMotions, self.play_motion_list_motions_service_name)
        self.get_motion_info_client = self.create_client(GetMotionInfo, self.play_motion_get_motion_info_service_name)

        self.play_motion_initialized = True
        return True

    def _get_motion_total_duration(self, motion_key: str) -> float:
        # --- ARREGLO MYPY --- Guardamos en variable local
        info_client = self.get_motion_info_client
        
        if info_client is None:
            return 0.0
        if not self._wait_for_service(info_client, timeout_sec=2.0):
            return 0.0

        request = GetMotionInfo.Request()
        request.motion_key = motion_key
        
        # Ahora usamos la variable local info_client
        future = info_client.call_async(request)
        
        if not self._wait_for_future(future, timeout_sec=2.0):
            return 0.0

        result = future.result()
        if result is None or result.motion is None:
            return 0.0

        times = getattr(result.motion, 'times_from_start', [])
        if times:
            return float(times[-1])
        return 0.0
    
    def get_available_actions(self) -> tuple[bool, Any]:
        if not self.is_connected:
            return False, "Robot no conectado."

        if not self._discover_play_motion_endpoints():
            return False, "No se encontró una interfaz válida de movimientos."

        # --- ARREGLO MYPY --- Guardamos en variable local
        list_client = self.list_motions_client
        
        if list_client is None:
            return False, "Servicio de listar movimientos no inicializado."

        if not self._wait_for_service(list_client, timeout_sec=3.0):
            return False, "Servicio de listar movimientos no disponible."

        request = ListMotions.Request()
        
        # Ahora usamos la variable local list_client
        future = list_client.call_async(request)
        
        if not self._wait_for_future(future, timeout_sec=4.0):
            return False, "Tiempo de espera agotado consultando movimientos."

        result = future.result()
        if result is None or not result.motion_keys:
            return False, "La lista de movimientos está vacía o hubo un error."

        return True, list(result.motion_keys)

    # --- ¡MÁGIA DE COMUNICACIÓN CON EL ROUTER! ---
    def set_action_feedback_callback(self, callback: Callable[[bool, int, str], None]):
        """Permite inyectar la función del router que mandará el WebSocket."""
        self._router_feedback_callback = callback

    def _play_motion_feedback_callback(self, feedback_msg: Any):
        """Disparado por ROS 2 en 2º plano cuando hay un avance en la acción."""
        try:
            # 1. Avisamos al Router de que seguimos (done_exec=False)
            if self._router_feedback_callback:
                self._router_feedback_callback(False, 0, "En progreso")
        except Exception as e:
            self.logger.error(f"Error procesando el feedback de la acción: {e}")

    def _play_motion_result_callback(self, future: Any):
        """Disparado por ROS 2 cuando el movimiento termina o falla."""
        try:
            result = future.result().result
            if result.success:
                status = "Acción completada con éxito"
                # Avisamos al Router de que ACABAMOS BIEN (done_exec=True)
                if self._router_feedback_callback:
                    self._router_feedback_callback(True, 100, status)
            else:
                status = f"Acción fallida: {result.error}"
                # Avisamos al Router del ERROR
                if self._router_feedback_callback:
                    self._router_feedback_callback(False, 0, status)
        except Exception as e:
            self.logger.error(f"Excepción en el resultado de la acción: {e}")
            if self._router_feedback_callback:
                self._router_feedback_callback(False, 0, "Excepción interna del robot")
        finally:
            with self._action_lock:
                self.current_goal_handle = None
                self.current_action_name = None

    def execute_action(self, action_type: str, target: str) -> tuple[bool, str]:
        if not self.is_connected:
            return False, "Robot no conectado."

        if not self._discover_play_motion_endpoints():
            return False, "Interfaz de acción no disponible."

        # --- ARREGLO MYPY --- 
        # Aunque ensure_play_motion_ready lo rellene, Mypy necesita verlo.
        act_client = self.play_motion_action_client
        
        if act_client is None:
            return False, "Cliente de acción no inicializado."

        goal_msg = PlayMotion2.Goal()
        goal_msg.motion_name = target
        goal_msg.skip_planning = False

        # Usamos act_client (variable local 100% segura que no es None)
        goal_future = act_client.send_goal_async(
            goal_msg,
            feedback_callback=self._play_motion_feedback_callback
        )

        if not self._wait_for_future(goal_future, timeout_sec=5.0):
            return False, "Timeout enviando la orden al robot."

        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            return False, "El robot ha rechazado realizar el movimiento."

        with self._action_lock:
            self.current_goal_handle = goal_handle
            self.current_action_name = target

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._play_motion_result_callback)

        return True, "Acción aceptada. Ejecutando..."
    
    def stop_action(self, action_type: str, target: str) -> bool:
        with self._action_lock:
            if self.current_goal_handle is None:
                return False
            if target != self.current_action_name:
                return False
            cancel_future = self.current_goal_handle.cancel_goal_async()

        if not self._wait_for_future(cancel_future, timeout_sec=5.0):
            return False

        cancel_response = cancel_future.result()
        if cancel_response.return_code == CancelResponse.ACCEPT:
            with self._action_lock:
                self.current_goal_handle = None
                self.current_action_name = None
            return True

        return False

    def get_teleop_topics(self) -> list[str]:
        topics_and_types = self.get_topic_names_and_types()
        safe_teleop_topics: list[str] = []
        safe_base_keywords = ['cmd_vel', 'cmd_vel_unstamped', 'cmd_vel_stamped', 'teleop', 'velocity', 'twist', 'diff_drive', 'base_controller', 'mobile_base_controller', 'wheel', 'drive', 'movement']

        def _topic_priority(topic_name: str) -> int:
            lower_name = topic_name.lower()
            for index, keyword in enumerate(safe_base_keywords):
                if keyword in lower_name:
                    return index
            return len(safe_base_keywords)

        for name, types in topics_and_types:
            if RosMsgTypes.TWIST in types or RosMsgTypes.TWIST_STAMPED in types:
                safe_teleop_topics.append(name)

        safe_teleop_topics.sort(key=lambda topic: (_topic_priority(topic), len(topic), topic))
        return safe_teleop_topics

    def get_camera_topics(self) -> list[str]:
        topics_and_types = self.get_topic_names_and_types()
        camera_topics: list[str] = []
        exclude_keywords = ['depth', 'disparity', 'mask', 'segmentation', 'semantic', 'instance', 'optical_flow', 'stereo']
        priority_keywords = ['camera', 'image_raw', 'rgb', 'color', 'front', 'main']

        def _camera_priority(topic_name: str) -> int:
            lower_name = topic_name.lower()
            for index, keyword in enumerate(priority_keywords):
                if keyword in lower_name:
                    return index
            return len(priority_keywords)

        for topic_name, types in topics_and_types:
            if RosMsgTypes.IMAGE in types or RosMsgTypes.COMPRESSED_IMAGE in types:
                lower_name = topic_name.lower()
                if any(excl in lower_name for excl in exclude_keywords):
                    continue
                camera_topics.append(topic_name)

        camera_topics.sort(key=lambda name: (_camera_priority(name), len(name), name))
        return camera_topics

    def get_robot_info(self) -> dict:
        hostname = socket.gethostname()
        domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
        
        battery_pct = self.latest_battery_pct
        e_stop_active = self.latest_estop_active
        is_charging = self.latest_is_charging
        
        has_base = False
        cameras_list: list[dict[str, str]] = []  
        has_manipulator = False
        has_head = False
        has_torso = False
        has_gripper = False
        has_lidar = False
        has_imu = False
        has_odom = False
        has_nav = False
        has_moveit = False
        has_ft_sensor = False
        has_play_motion = False

        detected_camera_roots = set()
        topics_and_types = self.get_topic_names_and_types()

        for topic_name, types in topics_and_types:
            topic_lower = topic_name.lower()
            
            if RosMsgTypes.TWIST in types or RosMsgTypes.TWIST_STAMPED in types:
                if any(keyword in topic_lower for keyword in DiscoveryConfig.BASE_KEYWORDS):
                    has_base = True
            
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
            
            if RosMsgTypes.LASER_SCAN in types:
                has_lidar = True
            elif RosMsgTypes.POINT_CLOUD2 in types:
                if not any(excl in topic_lower for excl in DiscoveryConfig.LIDAR_EXCLUDE_KEYWORDS):
                    has_lidar = True
            
            if RosMsgTypes.JOINT_TRAJ in types:
                if any(keyword in topic_lower for keyword in DiscoveryConfig.ARM_KEYWORDS):
                    has_manipulator = True
            
            if RosMsgTypes.IMU in types:
                has_imu = True
            
            if RosMsgTypes.ODOMETRY in types:
                has_odom = True
            
            if RosMsgTypes.JOINT_TRAJ in types:
                if any(keyword in topic_lower for keyword in DiscoveryConfig.GRIPPER_KEYWORDS):
                    has_gripper = True

            if any(keyword in topic_lower for keyword in ['torso', 'lift', 'spine', 'elevator']):
                has_torso = True

            if any(keyword in topic_lower for keyword in ['head', 'neck', 'pan_tilt', 'ptu']):
                has_head = True
            
            if RosMsgTypes.OCCUPANCY_GRID in types:
                has_nav = True
            
            if 'moveit_msgs/msg/PlanningScene' in types:
                has_moveit = True

            if 'geometry_msgs/msg/WrenchStamped' in types:
                if any(keyword in topic_lower for keyword in ['ft_sensor', 'wrench', 'force', 'torque']):
                    has_ft_sensor = True

            # Buscamos que exista el modulo de play_motion
            if 'play_motion' in topic_lower:
                has_play_motion = True

        teleop_topics: list[str] = []
        camera_topics: list[str] = []

        if has_base:
            teleop_topics = self.get_teleop_topics()

        if cameras_list:
            camera_topics = self.get_camera_topics()

         #has_base=False

        #cameras_list=[]

        #is_charging=True

        #has_ft_sensor=False

        #has_play_motion=False

        # --- INICIO PRUEBA (Sustituye las líneas de inicialización antiguas) ---

        #has_ft_sensor = self.latest_has_ft_sensor

        #has_play_motion = self.latest_has_play_motion

        #is_charging = self.latest_is_charging

        #has_base = self.latest_has_base

        # --- FIN PRUEBA --- 
        return {
            RobotInfoKeys.IDENTITY: {
                "hostname": hostname,
                "domain_id": domain_id,
            },
            RobotInfoKeys.STATUS: {
                "battery_pct": battery_pct,
                "e_stop_active": e_stop_active,
                "is_charging": is_charging
            },
            RobotInfoKeys.CAPABILITIES: {
                RobotInfoKeys.HAS_BASE: has_base,
                RobotInfoKeys.CAMERAS: cameras_list,
                RobotInfoKeys.TELEOP_TOPICS: teleop_topics,
                RobotInfoKeys.CAMERA_TOPICS: camera_topics,
                RobotInfoKeys.HAS_MANIPULATOR: has_manipulator,
                RobotInfoKeys.HAS_HEAD: has_head,
                RobotInfoKeys.HAS_TORSO: has_torso,
                RobotInfoKeys.HAS_GRIPPER: has_gripper,
                RobotInfoKeys.HAS_IMU: has_imu,
                RobotInfoKeys.HAS_ODOMETRY: has_odom,
                RobotInfoKeys.HAS_LIDAR: has_lidar,
                RobotInfoKeys.HAS_NAV: has_nav,
                RobotInfoKeys.HAS_MOVEIT: has_moveit,
                RobotInfoKeys.HAS_FT_SENSOR: has_ft_sensor,
                RobotInfoKeys.HAS_PLAY_MOTION: has_play_motion,
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
        
        self.gateway_node = TiagoBridgeNode()
        self.safety_node = SafetyFilterNode()
        
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.gateway_node)
        self.executor.add_node(self.safety_node)
        
        self._is_running = True

        self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self.spin_thread.start()

    def _spin_loop(self):
        try:
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
            if self.safety_node:
                self.safety_node.stop_robot()
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

    def connect_to_robot(self) -> int:
        if self._is_running and self.gateway_node:
            return self.gateway_node.connect()
        return 0

    def disconnect_from_robot(self):
        if self._is_running and self.gateway_node:
            self.gateway_node.disconnect()
        
    def stop_robot(self):
        if self._is_running and self.gateway_node:
            self.gateway_node.stop_robot()

    def set_control_mode(self, event: str, topic: str = "cmd_vel") -> tuple[bool, str]:
        if self._is_running and self.gateway_node and self.safety_node:
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
        if self._is_running and self.gateway_node:
            return self.gateway_node.check_connection_silently()
        return 0
    
    def is_video_server_running(self) -> bool:
        if self._is_running and self.gateway_node:
            return self.gateway_node.check_video_server_silently()
        return False
    
    def is_topic_active(self, topic_name: str) -> bool:
        if self._is_running and self.gateway_node:
            return self.gateway_node.is_topic_active(topic_name)
        return False
    
    def get_teleop_topics(self) -> list[str]:
        if self._is_running and self.gateway_node:
            return self.gateway_node.get_teleop_topics()
        return []

    def get_camera_topics(self) -> list[str]:
        if self._is_running and self.gateway_node:
            return self.gateway_node.get_camera_topics()
        return []
    
    def get_robot_capabilities(self) -> dict:
        if self._is_running and self.gateway_node:
            return self.gateway_node.get_robot_info()
        return {}

    # ==========================================
    # NUEVA API PARA EL ROUTER (ACTIONS)
    # ==========================================
    def set_action_feedback_callback(self, callback: Callable[[bool, int, str], None]):
        """Pasa la función inyectada al nodo de ROS 2."""
        if self._is_running and self.gateway_node:
            self.gateway_node.set_action_feedback_callback(callback)

    def get_available_actions(self) -> tuple[bool, Any]:
        if self._is_running and self.gateway_node:
            return self.gateway_node.get_available_actions()
        return False, "El subsistema ROS 2 no está corriendo."

    def execute_action(self, action_type: str, target: str) -> tuple[bool, str]:
        if self._is_running and self.gateway_node:
            return self.gateway_node.execute_action(action_type, target)
        return False, "El subsistema ROS 2 no está corriendo."

    def stop_action(self, action_type: str, target: str) -> bool:
        if self._is_running and self.gateway_node:
            return self.gateway_node.stop_action(action_type, target)
        return False