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
import xml.etree.ElementTree as ET # ¡NUEVO! Para leer el URDF (XML)
from rclpy.timer import Timer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy # ¡NUEVO! Para leer topics "latched"
from std_msgs.msg import String # ¡NUEVO! Para el topic /robot_description
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # ¡NUEVO! Para mover motores
from typing import Any, Optional, Callable # <-- ¡NUEVO! Importamos Callable
from rclpy.node import Node  # type: ignore[import]
from geometry_msgs.msg import Twist # type: ignore[import]
from rclpy.executors import SingleThreadedExecutor # type: ignore[import]
from rclpy.action import ActionClient, CancelResponse # type: ignore[import]
from sensor_msgs.msg import BatteryState, JointState, LaserScan, Imu, Range, PointCloud2, NavSatFix, Temperature # ¡Añadidos NavSatFix y Temperature! # type: ignore[import]
from nav_msgs.msg import Odometry # ¡NUEVO!
from geometry_msgs.msg import WrenchStamped # ¡NUEVO! # type: ignore[import] 
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
        self._router_feedback_callback: Optional[Callable[[bool, bool, int, str], None]] = None

        # ==========================================
        # ¡NUEVO! Declaramos las variables del cronómetro aquí para que Mypy sea feliz
        # ==========================================
        self.action_progress_timer: Optional[Timer] = None
        self.action_start_time: float = 0.0
        self.current_motion_total_duration: float = 0.0
        self._fake_progress: int = 0

        # ==========================================
        # ¡NUEVO! LÓGICA DE ARTICULACIONES Y URDF
        # ==========================================
        self.joint_limits: list[dict[str, Any]] = []
        self.joint_publishers: dict[str, Any] = {}

        self.dynamic_controllers: dict[str, list[str]] = {} # ¡NUEVO! Guarda las relaciones automáticamente
        
        # ¡NUEVO! Memoria para evitar bucles infinitos de comandos
        self._visited_controllers: set[str] = set()
        
        self.current_joint_states: dict[str, float] = {}
        self.create_subscription(JointState, '/joint_states', self._joint_states_callback, 10)

        # ==========================================
        # ¡NUEVO! MEMORIA DE SENSORES ACTIVOS
        # ==========================================
        # Formato: { "/scan": {"sub": Subscription, "callback": Callable, "last_sent": float} }
        self.active_sensor_streams: dict[str, dict[str, Any]] = {}

        # ==========================================
        # ¡NUEVO! ESCUDO ANTICOLISIÓN (Segundo Plano)
        # ==========================================
        self.safety_lidar_topics: set[str] = set()
        self.imminent_collisions: dict[str, bool] = {}

        # ¡NUEVO! Chivato de seguridad
        self.safety_alert = "OK"
        self.create_subscription(String, 'web_teleop/safety_alert', self._safety_alert_callback, 10)
        
        # Nos suscribimos al URDF. Usamos TRANSIENT_LOCAL porque este topic a veces
        # solo se publica una vez al arrancar el robot y se queda "guardado" en la red.
        qos_urdf = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.urdf_sub = self.create_subscription(String, '/robot_description', self._urdf_callback, qos_urdf)

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

    def _safety_alert_callback(self, msg: String):
        self.safety_alert = msg.data
    
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
            
            # ==========================================
            # ¡NUEVO! Auto-descubrimiento del Escudo Anticolisión
            # ==========================================
            if 'sensor_msgs/msg/LaserScan' in types:
                if name not in self.safety_lidar_topics:
                    self.safety_lidar_topics.add(name)
                    
                    # Fábrica para saber de qué topic viene la alerta
                    def make_safety_callback(t_name: str) -> Callable[[LaserScan], None]:
                        def cb(msg: LaserScan) -> None:
                            self._safety_lidar_callback(t_name, msg)
                        return cb
                        
                    self.create_subscription(LaserScan, name, make_safety_callback(name), QoSProfile(depth=1))
                    self.logger.info(f"🛡️ Escudo Anticolisión activado en segundo plano: {name}")

            # ¡NUEVO! Detectar controladores de articulaciones universalmente
            if 'trajectory_msgs/msg/JointTrajectory' in types:
                if name not in self._visited_controllers:
                    # ¡LA CLAVE! Lo marcamos como visitado ANTES de lanzar el comando
                    # Así, si el comando falla, no entraremos en un bucle infinito.
                    self._visited_controllers.add(name)
                    
                    # Lanzamos un hilo rápido para que le pregunte a ROS 2 sin bloquear el servidor
                    threading.Thread(target=self._resolve_controller_joints, args=(name,), daemon=True).start()
    
    def _resolve_controller_joints(self, topic_name: str):
        """Usa comandos de terminal en 2º plano para preguntarle a ROS 2 qué motores posee este controlador."""
        import subprocess
        import ast
        
        parts = topic_name.split('/')
        if len(parts) < 3: return
        
        controller_name = parts[1]
        joints_list = []
        
        # Posibles respuestas según la versión de ROS 2
        target_phrases = ["String array value is:", "String values are:"]

        def extract_joints(output_text: str) -> list:
            for phrase in target_phrases:
                if phrase in output_text:
                    # Cortamos todo lo que haya antes (incluido el error de CycloneDDS)
                    list_str = output_text.split(phrase)[1].strip()
                    try:
                        return ast.literal_eval(list_str)
                    except Exception:
                        pass
            return []
        
        # Intento 1: Arquitectura estándar ros2_control (/controller_manager)
        try:
            res = subprocess.check_output(
                ['ros2', 'param', 'get', '/controller_manager', f'{controller_name}.joints'],
                text=True, timeout=3.0, stderr=subprocess.DEVNULL
            )
            joints_list = extract_joints(res)
        except Exception:
            pass
            
        # Intento 2: Nodos de acción independientes (¡El que usa tu TIAGo!)
        if not joints_list:
            try:
                res = subprocess.check_output(
                    ['ros2', 'param', 'get', f'/{controller_name}', 'joints'],
                    text=True, timeout=3.0, stderr=subprocess.DEVNULL
                )
                joints_list = extract_joints(res)
            except Exception:
                pass
                
        if joints_list:
            self.dynamic_controllers[topic_name] = joints_list
            self.logger.info(f"🤖 Controlador Universal detectado: '{topic_name}' maneja {len(joints_list)} motores.")
    
    def _battery_callback(self, msg: BatteryState):
        if msg.percentage <= 1.0:
            self.latest_battery_pct = float(msg.percentage * 100.0)
        else:
            self.latest_battery_pct = float(msg.percentage)

    def _charging_callback(self, msg: Bool):
        self.latest_is_charging = msg.data

    def _estop_callback(self, msg: Bool):
        self.latest_estop_active = msg.data

    def _joint_states_callback(self, msg: JointState):
        """Memoriza la posición actual de todos los motores en tiempo real."""
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_states[name] = pos

    # ==========================================
    # ¡NUEVO! LECTURA DE URDF Y PUERTA RÁPIDA
    # ==========================================
    def _urdf_callback(self, msg: String):
        """Lee el XML del robot y extrae los límites de los motores reales."""
        if self.joint_limits: # Si ya los hemos leído, no hacemos nada más
            return
        try:
            root = ET.fromstring(msg.data)
            limits = []
            for joint in root.findall('joint'):
                j_type = joint.get('type')
                j_name = joint.get('name')
                # Solo queremos articulaciones que se muevan con límites
                if j_type in ['revolute', 'prismatic'] and j_name:
                    limit_tag = joint.find('limit')
                    if limit_tag is not None:
                        try:
                            lower = float(limit_tag.get('lower', 0.0))
                            upper = float(limit_tag.get('upper', 0.0))
                            limits.append({"name": j_name, "min": lower, "max": upper})
                        except ValueError:
                            pass
            self.joint_limits = limits
            self.logger.info(f"¡URDF escaneado con éxito! {len(limits)} motores detectados.")
        except Exception as e:
            self.logger.error(f"Error parseando el URDF del robot: {e}")

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
            self.logger.info("Desconectando del Tiago. Aplicando freno de emergencia global.")
            
            # ==========================================
            # ¡NUEVO! CANCELACIÓN SEGURA DE PLAYMOTION
            # Si hay un movimiento ejecutándose, lo matamos.
            # ==========================================
            with self._action_lock:
                if getattr(self, 'current_goal_handle', None) is not None:
                    self.logger.info("Cancelando movimiento de PlayMotion en curso por desconexión...")
                    try:
                        # Levantamos el banderín por si el callback residual salta
                        self.was_canceled_by_user = True 
                        # Mandamos la orden de cancelar a ROS 2
                        self.current_goal_handle.cancel_goal_async()
                    except Exception as e:
                        self.logger.warning(f"Aviso al cancelar acción durante desconexión: {e}")
                    finally:
                        self.current_goal_handle = None
                        self.current_action_name = None

            # Apagamos también nuestro cronómetro interno por si acaso
            if hasattr(self, 'action_progress_timer') and self.action_progress_timer:
                self.action_progress_timer.cancel()
            
            # ==========================================
            # ¡LA SOLUCIÓN AL SPAM! Limpieza de Sensores
            # Destruimos todos los suscriptores para que dejen de mandar datos a un socket muerto
            # ==========================================
            if hasattr(self, 'active_sensor_streams'):
                topics_to_close = list(self.active_sensor_streams.keys())
                for topic in topics_to_close:
                    self.stop_sensor_stream(topic)
            
            # El freno de las ruedas que ya tenías
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
    
    def set_control_mode(self, event: str, control_type: str = "TELEOP", topic: str = "cmd_vel") -> tuple[bool, str]:
        if not self.is_connected:
            return False, "Robot no conectado lógicamente."

        if event == ControlEvent.START:
            self.is_control_active = True
            
            # Si es el joystick, validamos el topic de las ruedas
            if control_type != "JOINT":
                is_valid, msg = self.validate_topic(topic)
                if not is_valid:
                    self.is_control_active = False
                    return False, msg
                return True, msg
            
            # Si son articulaciones, damos luz verde directamente
            return True, "JOINT_CONTROL"
            
        elif event == ControlEvent.STOP:
            self.is_control_active = False
            # Solo frenamos las ruedas si estábamos usándolas
            if control_type != "JOINT":
                self.stop_robot()
            return True, ""
            
        return False, "Evento desconocido."

    # ¡NUEVO! Analizador rápido del láser
    def _safety_lidar_callback(self, topic_name: str, msg: LaserScan):
        # Distancia mínima de seguridad (35 cm). Ajustable según el tamaño de tu robot.
        SAFE_DIST = 0.35 
        collision = False
        for r in msg.ranges:
            # Ignoramos rebotes contra el propio chasis (0.0 a 0.05) y lecturas infinitas
            if 0.05 < r < SAFE_DIST:
                collision = True
                break
        self.imminent_collisions[topic_name] = collision

    def publish_velocity(self, v: float, w: float) -> tuple[bool, str]: # ¡Cambio a Tupla!
        if not self.is_connected or not self.is_control_active:
            return False, "Control no activo."

        # ¡LA LECTURA DEL CHIVATO!
        if v > 0.0 and self.safety_alert in ["FRONT", "BOTH"]:
            return False, "⚠️ ANTICOLISIÓN: Obstáculo detectado en la dirección de avance."
        if v < 0.0 and self.safety_alert in ["REAR", "BOTH"]:
            return False, "⚠️ ANTICOLISIÓN: Obstáculo detectado en la dirección de retroceso."

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        
        self.vel_publisher.publish(msg)
        return True, ""

    def stop_robot(self):
        try:
            if rclpy.ok() and self.vel_publisher is not None:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.vel_publisher.publish(msg)
        except Exception as e:
            self.logger.debug(f"Freno omitido: {e}")

    #FUNCION PARA MOVER ARTICULACIONES EN TIEMPO REAL (BARRA DESLIZADORA)
    def publish_joint_position(self, joint_name: str, value: float) -> bool:
        if not self.is_connected or not self.is_control_active:
            return False

        # 1. SEGURIDAD: Recortar a los límites URDF
        if self.joint_limits:
            for limit in self.joint_limits:
                if limit["name"] == joint_name:
                    value = max(limit["min"], min(limit["max"], value))
                    break

        # ==========================================
        # 2. BÚSQUEDA DINÁMICA UNIVERSAL
        # ==========================================
        target_topic = None
        expected_joints = []
        
        for topic, joints in self.dynamic_controllers.items():
            if joint_name in joints:
                target_topic = topic
                expected_joints = joints
                break
                
        if not target_topic:
            self.logger.warning(f"Aún no he descubierto a qué controlador pertenece: {joint_name}")
            return False

        if target_topic not in self.joint_publishers:
            self.joint_publishers[target_topic] = self.create_publisher(JointTrajectory, target_topic, 10)

        # ==========================================
        # 3. CONSTRUCCIÓN DEL PAQUETE COMPLETO
        # ==========================================
        msg = JointTrajectory()
        msg.joint_names = expected_joints
        
        point = JointTrajectoryPoint()
        point.positions = []
        
        for j_name in expected_joints:
            if j_name == joint_name:
                point.positions.append(float(value))
            else:
                current_pos = self.current_joint_states.get(j_name, 0.0)
                point.positions.append(float(current_pos))
                
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000 # 200 milisegundos
        
        msg.points = [point]
        self.joint_publishers[target_topic].publish(msg)
        return True

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
        info_client = self.get_motion_info_client
        if info_client is None:
            return 0.0
        if not self._wait_for_service(info_client, timeout_sec=2.0):
            return 0.0

        request = GetMotionInfo.Request()
        request.motion_key = motion_key
        future = info_client.call_async(request)
        
        if not self._wait_for_future(future, timeout_sec=2.0):
            return 0.0

        result = future.result()
        if result is None or result.motion is None:
            return 0.0

        times = getattr(result.motion, 'times_from_start', [])
        if times:
            # ¡ARREGLO DE LA BARRA! Le sumamos 2.5s de tiempo de planificación del robot
            return float(times[-1]) + 2.5 
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
    def set_action_feedback_callback(self, callback: Callable[[bool, bool, int, str], None]):
        self._router_feedback_callback = callback

    """
    def _play_motion_feedback_callback(self, feedback_msg: Any):
        Disparado por ROS 2 en 2º plano cuando hay un avance en la acción.
        try:
            # 🕵️ CHIVATO 2: Ver la respuesta cruda de ROS 2 mientras se mueve
            self.logger.info(f"[DEBUG ROS2] Feedback crudo recibido: {feedback_msg}")

            # 1. Calculamos el tiempo que llevamos
            current_time = feedback_msg.feedback.current_time
            elapsed = float(current_time.sec) + float(current_time.nanosec) * 1e-9
            
            # 2. Calculamos el porcentaje
            progress = 0
            if hasattr(self, 'current_motion_total_duration') and self.current_motion_total_duration > 0:
                progress = int((elapsed / self.current_motion_total_duration) * 100)
                progress = min(99, max(0, progress)) # Lo topamos al 99% para que el 100% sea al terminar

            # 3. Avisamos al Router
            if self._router_feedback_callback:
                self._router_feedback_callback(False, progress, f"Progreso: {progress}%")
        except Exception as e:
            self.logger.error(f"Error procesando el feedback de la acción: {e}") 
    """

    # ¡NUEVO! Nuestro propio reloj interno para forzar la barra de progreso (VERSIÓN LIMPIA)
    def _progress_timer_callback(self):
        try:
            # Si ya no hay acción en curso, apagamos el cronómetro
            if not self.current_goal_handle:
                if self.action_progress_timer:
                    self.action_progress_timer.cancel()
                return

            # Calculamos cuánto tiempo ha pasado desde que pulsamos el botón
            elapsed = time.time() - self.action_start_time
            total_dur = self.current_motion_total_duration

            if total_dur > 0:
                # Matemática exacta: (Tiempo transcurrido / Duración total) * 100
                calc_prog = int((elapsed / total_dur) * 100)
                progress = min(95, max(0, calc_prog)) # Lo topamos al 95% para que pegue el salto a 100% al terminar
            else:
                # Si falla la extracción de tiempo, avanzamos de forma lenta artificial
                self._fake_progress += 5
                progress = min(95, self._fake_progress)

            # Enviamos el progreso al móvil
            if self._router_feedback_callback:
                self._router_feedback_callback(True, False, progress, f"Progreso: {progress}%")
        except Exception as e:
            self.logger.error(f"Error en timer de progreso: {e}")

    def _play_motion_result_callback(self, future: Any):
        """Disparado por ROS 2 cuando el movimiento termina o falla."""
        if self.action_progress_timer:
            self.action_progress_timer.cancel()
        
        try:
            result = future.result().result

            # ¡NUEVA LÓGICA! Comprobamos primero nuestro banderín manual
            if getattr(self, 'was_canceled_by_user', False):
                self.was_canceled_by_user = False # Limpiamos el banderín
                if self._router_feedback_callback:
                    self._router_feedback_callback(True, True, 100, "Acción detenida por el usuario")
                    
            elif result.success:
                if self._router_feedback_callback:
                    self._router_feedback_callback(True, True, 100, "Acción completada con éxito")
            else:
                error_msg = str(getattr(result, 'error', 'Error desconocido'))
                if "cancel" in error_msg.lower():
                    if self._router_feedback_callback:
                        self._router_feedback_callback(True, True, 100, "Acción detenida por el usuario")
                else:
                    if self._router_feedback_callback:
                        self._router_feedback_callback(False, True, 0, f"Acción fallida: {error_msg}")
        except Exception as e:
            self.logger.error(f"Excepción en el resultado de la acción: {e}")
            if self._router_feedback_callback:
                self._router_feedback_callback(False, True, 0, "Excepción interna del robot")
        finally:
            with self._action_lock:
                self.current_goal_handle = None
                self.current_action_name = None

    """
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

        self.current_motion_total_duration = self._get_motion_total_duration(target)
        
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
    """

    def execute_action(self, action_type: str, target: str) -> tuple[bool, str]:
        if not self.is_connected:
            return False, "Robot no conectado."
        if not self._discover_play_motion_endpoints():
            return False, "Interfaz de acción no disponible."

        act_client = self.play_motion_action_client
        if act_client is None:
            return False, "Cliente de acción no inicializado."

        # ¡NUEVO! Inicializamos tiempos y arrancamos nuestro reloj
        self._fake_progress = 0
        self.current_motion_total_duration = self._get_motion_total_duration(target)
        self.action_start_time = time.time()
        
        # Apagamos cronómetros fantasmas anteriores si los hubiera
        if self.action_progress_timer:
            self.action_progress_timer.cancel()
        
        # Le decimos a ROS 2 que llame a nuestro cronómetro cada 0.25 segundos
        self.action_progress_timer = self.create_timer(0.25, self._progress_timer_callback)
        
        goal_msg = PlayMotion2.Goal()
        goal_msg.motion_name = target
        goal_msg.skip_planning = False

        goal_future = act_client.send_goal_async(
            goal_msg
            # ¡Ya no necesitamos el feedback callback de ROS 2 porque usamos el nuestro!
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
            
            # ¡BANDERÍN! Levantamos la mano para que el callback sepa que fuimos nosotros
            self.was_canceled_by_user = True 
            
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
    
    def get_available_sensors(self) -> list[dict[str, str]]:
        """
        Escanea la red de ROS 2 y devuelve una lista de diccionarios con los sensores detectados.
        Formato: [{"topic": "/scan_front_raw", "type": "LaserScan"}, ...]
        """
        topics_and_types = self.get_topic_names_and_types()
        sensors_list = []
        
        # Nuestro Diccionario Traductor Ampliado con los datos de tu TIAGo
        # Nuestro Diccionario Traductor Ampliado
        supported_types = {
            RosMsgTypes.LASER_SCAN: "LaserScan",
            RosMsgTypes.IMU: "Imu",
            RosMsgTypes.BATTERY: "BatteryState",
            RosMsgTypes.RANGE: "Range",
            RosMsgTypes.POINT_CLOUD2: "PointCloud2",
            
            # ==========================================
            # ¡NUEVOS SENSORES UNIVERSALES!
            # ==========================================
            RosMsgTypes.ODOMETRY: "Odometry",
            RosMsgTypes.NAV: "NavSatFix",
            RosMsgTypes.WRENCH: "Wrench",
            RosMsgTypes.TEMPERATURE: "Temperature"
        }
        
        # Filtro de limpieza: Ignorar topics de procesamiento intermedio para no saturar el menú
        exclude_keywords = ['throttle', 'filtered', 'depth/points']
        
        for topic_name, types in topics_and_types:
            topic_lower = topic_name.lower()
            
            # Si el topic contiene palabras de procesamiento interno, lo ignoramos
            if any(excl in topic_lower for excl in exclude_keywords):
                continue
                
            # Comprobamos si es un sensor de nuestro diccionario
            for ros_type, clean_name in supported_types.items():
                if ros_type in types:
                    sensors_list.append({
                        "topic": topic_name,
                        "type": clean_name
                    })
                    break 
                    
        # Ordenamos la lista alfabéticamente por tipo, y luego por nombre del topic
        sensors_list.sort(key=lambda x: (x["type"], x["topic"]))
        return sensors_list
    
    # ==========================================
    # ¡NUEVO! LÓGICA DE SUSCRIPCIÓN DE SENSORES
    # ==========================================
    def start_sensor_stream(self, topic: str, callback: Callable) -> bool:
        """Busca el tipo del topic, crea un suscriptor y empieza a emitir datos."""
        topics_and_types = self.get_topic_names_and_types()
        topic_type = None
        
        for name, types in topics_and_types:
            if name == topic:
                topic_type = types[0] # Cogemos el tipo principal (Ej: sensor_msgs/msg/LaserScan)
                break
                
        if not topic_type:
            self.logger.error(f"No se pudo suscribir: El topic {topic} no existe.")
            return False
            
        # Si ya lo estamos escuchando, no hacemos nada
        if topic in self.active_sensor_streams:
            return True 

        # ==========================================
        # ¡SOLUCIÓN MYPY! Fábrica de callbacks tipada
        # ==========================================
        def make_callback(t: str, s_type: str) -> Callable[[Any], None]:
            def callback(msg: Any) -> None:
                self._sensor_callback(t, msg, s_type)
            return callback

        sub = None
        # Según el tipo, usamos nuestra fábrica para crear el callback perfecto
        if topic_type == RosMsgTypes.LASER_SCAN:
            sub = self.create_subscription(LaserScan, topic, make_callback(topic, "LaserScan"), 10)
        elif topic_type == RosMsgTypes.IMU:
            sub = self.create_subscription(Imu, topic, make_callback(topic, "Imu"), 10)
        elif topic_type == RosMsgTypes.BATTERY:
            sub = self.create_subscription(BatteryState, topic, make_callback(topic, "BatteryState"), 10)
        elif topic_type == RosMsgTypes.RANGE:
            sub = self.create_subscription(Range, topic, make_callback(topic, "Range"), 10)
        elif topic_type == RosMsgTypes.POINT_CLOUD2:
            sub = self.create_subscription(PointCloud2, topic, make_callback(topic, "PointCloud2"), 10)
        # --- ¡LOS NUEVOS! ---
        elif topic_type == RosMsgTypes.ODOMETRY:
            sub = self.create_subscription(Odometry, topic, make_callback(topic, "Odometry"), 10)
        elif topic_type == RosMsgTypes.NAV:
            sub = self.create_subscription(NavSatFix, topic, make_callback(topic, "NavSatFix"), 10)
        elif topic_type == RosMsgTypes.WRENCH:
            sub = self.create_subscription(WrenchStamped, topic, make_callback(topic, "Wrench"), 10)
        elif topic_type == RosMsgTypes.TEMPERATURE:
            sub = self.create_subscription(Temperature, topic, make_callback(topic, "Temperature"), 10)
        else:
            self.logger.error(f"Tipo de sensor no soportado: {topic_type}")
            return False
            
        # Guardamos el grifo en memoria con su reloj a cero
            
        # Guardamos el grifo en memoria con su reloj a cero
        self.active_sensor_streams[topic] = {
            "sub": sub,
            "callback": callback,
            "last_sent": 0.0
        }
        self.logger.info(f"Suscripción a sensor '{topic}' iniciada.")
        return True

    def stop_sensor_stream(self, topic: str):
        """Destruye el suscriptor y para el flujo de datos."""
        if topic in self.active_sensor_streams:
            stream_data = self.active_sensor_streams.pop(topic)
            self.destroy_subscription(stream_data["sub"])
            self.logger.info(f"Suscripción a sensor '{topic}' destruida.")

    def _sensor_callback(self, topic: str, msg: Any, sensor_type: str):
        """El Traductor: Convierte bytes de ROS 2 en JSON y frena la velocidad (Throttler)."""
        stream_data = self.active_sensor_streams.get(topic)
        if not stream_data: return
        
        current_time = time.time()
        # EL FRENO: Si hace menos de 0.1 segundos (10 Hz) que mandamos el último paquete, lo ignoramos.
        # Sensores como la IMU publican a 100Hz o 200Hz. Esto salva el router Wi-Fi.
        if current_time - stream_data["last_sent"] < 0.1:
            return
            
        stream_data["last_sent"] = current_time
        
        json_data = {"topic": topic, "type": sensor_type, "data": {}}
        
        try:
            if sensor_type == "LaserScan":
                # ESTRATEGIA DE COMPRESIÓN PARA MÓVILES:
                
                # 1. Diezmado (Downsampling): Si el láser es muy denso (>500 puntos), 
                # cogemos 1 de cada 3 puntos. Si es pequeño, lo mandamos entero.
                step = 3 if len(msg.ranges) > 500 else 1
                
                # 2. Redondeo y limpieza: Redondeamos a 2 decimales y quitamos los 'inf'
                max_r = round(float(msg.range_max), 2)
                safe_ranges = []
                for r in msg.ranges[::step]: # [::step] es la magia de Python para saltar elementos
                    if r == float('inf') or r != r: # r != r detecta NaN
                        safe_ranges.append(max_r)
                    else:
                        safe_ranges.append(round(float(r), 2))
                
                json_data["data"] = {
                    "angle_min": msg.angle_min,
                    "angle_max": msg.angle_max,
                    # Como nos saltamos puntos, el incremento angular ahora es mayor
                    "angle_increment": msg.angle_increment * step, 
                    "range_min": msg.range_min,
                    "range_max": msg.range_max,
                    "ranges": safe_ranges
                }
            elif sensor_type == "Imu":
                json_data["data"] = {
                    "orientation": {"x": msg.orientation.x, "y": msg.orientation.y, "z": msg.orientation.z, "w": msg.orientation.w},
                    "angular_velocity": {"x": msg.angular_velocity.x, "y": msg.angular_velocity.y, "z": msg.angular_velocity.z},
                    "linear_acceleration": {"x": msg.linear_acceleration.x, "y": msg.linear_acceleration.y, "z": msg.linear_acceleration.z}
                }
            elif sensor_type == "BatteryState":
                json_data["data"] = {
                    "voltage": msg.voltage,
                    "percentage": msg.percentage * 100.0 if msg.percentage <= 1.0 else msg.percentage,
                    "power_supply_status": msg.power_supply_status
                }
            elif sensor_type == "Range":
                json_data["data"] = {
                    "range": msg.range,
                    "min_range": msg.min_range,
                    "max_range": msg.max_range,
                    "field_of_view": msg.field_of_view
                }
            elif sensor_type == "PointCloud2":
                # NUBE DE PUNTOS: Son millones de bytes. No la mandamos cruda al móvil,
                # solo mandamos los metadatos para que Android sepa que funciona pero no se cuelgue.
                json_data["data"] = {
                    "width": msg.width,
                    "height": msg.height,
                    "is_dense": msg.is_dense,
                    "note": "PointCloud2 es demasiado masivo para el móvil. Solo se envían metadatos."
                }
            elif sensor_type == "PointCloud2":
                json_data["data"] = {
                    "width": msg.width, "height": msg.height, "is_dense": msg.is_dense,
                    "note": "PointCloud2 es demasiado masivo para el móvil. Solo se envían metadatos."
                }
                
            # --- ¡LOS NUEVOS TRADUCTORES A JSON! ---
            elif sensor_type == "Odometry":
                json_data["data"] = {
                    "position": {"x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y},
                    "linear_velocity": msg.twist.twist.linear.x,
                    "angular_velocity": msg.twist.twist.angular.z
                }
            elif sensor_type == "NavSatFix":
                json_data["data"] = {
                    "latitude": msg.latitude,
                    "longitude": msg.longitude,
                    "altitude": msg.altitude,
                    "status": msg.status.status # Indica si tiene cobertura de satélites
                }
            elif sensor_type == "Wrench":
                json_data["data"] = {
                    "force": {"x": msg.wrench.force.x, "y": msg.wrench.force.y, "z": msg.wrench.force.z},
                    "torque": {"x": msg.wrench.torque.x, "y": msg.wrench.torque.y, "z": msg.wrench.torque.z}
                }
            elif sensor_type == "Temperature":
                json_data["data"] = {
                    "temperature": msg.temperature # En grados Celsius
                }
                
            # Disparamos los datos limpios hacia el router web
            stream_data["callback"](json_data)
            
        except Exception as e:
            self.logger.error(f"Error traduciendo datos del sensor {topic}: {e}")

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

        # ==========================================
        # ¡NUEVO! Sincronización del estado real de los motores
        # ==========================================
        updated_joint_limits = []
        for limit in self.joint_limits:
            limit_copy = limit.copy()
            # Leemos la posición actual de nuestra memoria (o 0.0 si aún no ha llegado)
            limit_copy["current_value"] = self.current_joint_states.get(limit["name"], 0.0)
            updated_joint_limits.append(limit_copy)
        
        #self.logger.info(f"ENVIANDO LÍMITES AL MÓVIL: {updated_joint_limits}")

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
                RobotInfoKeys.CONTROLABLE_JOINTS: updated_joint_limits #QUE ARTICULACIONES PUEDO MOVER Y SUS LIMITES
                
            }
        }
    
    # ==========================================
    # FUNCIONES DE INSPECCIÓN DE RED (ESTILO ROS2CLI)
    # ==========================================
    def get_all_topics(self) -> dict[str, list[str]]:
        """Devuelve un diccionario { '/topic_name': ['type1', 'type2'] }"""
        return {name: types for name, types in self.get_topic_names_and_types()}

    def get_all_services(self) -> dict[str, list[str]]:
        """Devuelve un diccionario { '/service_name': ['type1', 'type2'] }"""
        return {name: types for name, types in self.get_service_names_and_types()}

    def get_all_actions(self) -> dict[str, list[str]]:
        """Devuelve un diccionario { '/action_name': ['type1', 'type2'] }"""
        try:
            from rclpy.action import get_action_names_and_types
            return {name: types for name, types in get_action_names_and_types(self)}
        except Exception as e:
            self.logger.warning(f"Error obteniendo la lista de actions: {e}")
            return {}


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

    def set_control_mode(self, event: str, control_type: str = "TELEOP", topic: str = "cmd_vel") -> tuple[bool, str]:
        if self._is_running and self.gateway_node and self.safety_node:
            # Le pasamos el tipo al nodo puente
            success, msg = self.gateway_node.set_control_mode(event, control_type, topic)
            
            if success and event == ControlEvent.START:
                # ¡LA MAGIA ESTÁ AQUÍ! 
                # Si es un Joystick, activamos el Watchdog de seguridad.
                if control_type != "JOINT":
                    self.safety_node.set_target_topic(msg)
                else:
                    self.logger.info("Modo JOINT activado: Watchdog de ruedas desactivado por seguridad.")
            return success, msg
        return False, "El subsistema ROS 2 no está corriendo."

    def publish_velocity(self, v: float, w: float) -> tuple[bool, str]:
        if self._is_running and self.gateway_node:
            return self.gateway_node.publish_velocity(v, w)
        return False, "Subsistema ROS 2 apagado."
    
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
    
    # ¡NUEVO! Puente para el radar de sensores
    def get_available_sensors(self) -> list[dict[str, str]]:
        if self._is_running and self.gateway_node:
            return self.gateway_node.get_available_sensors()
        return []
    
    def start_sensor_stream(self, topic: str, callback: Callable) -> bool:
        if self._is_running and self.gateway_node:
            return self.gateway_node.start_sensor_stream(topic, callback)
        return False
        
    def stop_sensor_stream(self, topic: str):
        if self._is_running and self.gateway_node:
            self.gateway_node.stop_sensor_stream(topic)
    
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
    
    def publish_joint_position(self, joint_name: str, value: float) -> bool:
        """Puente para enviar posiciones a las articulaciones (Puerta Rápida)."""
        if self._is_running and self.gateway_node:
            return self.gateway_node.publish_joint_position(joint_name, value)
        return False
    
    # ==========================================
    # INSPECCIÓN DE RED
    # ==========================================
    def get_all_topics(self) -> dict:
        if self._is_running and self.gateway_node:
            return self.gateway_node.get_all_topics()
        return {}

    def get_all_services(self) -> dict:
        if self._is_running and self.gateway_node:
            return self.gateway_node.get_all_services()
        return {}

    def get_all_actions(self) -> dict:
        if self._is_running and self.gateway_node:
            return self.gateway_node.get_all_actions()
        return {}