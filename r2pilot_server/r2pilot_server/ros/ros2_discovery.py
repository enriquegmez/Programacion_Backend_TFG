## @file ros2_discovery.py
#  @brief Gestor de descubrimiento de hardware y topología para ROS 2.
#  @details Aisla la lógica de parseo del URDF, resolución de controladores 
#           y construcción del diccionario de capacidades del robot.
#  @author Enrique Gómez
#  @date 2026

import os
import socket
import logging
import threading
import subprocess
import xml.etree.ElementTree as ET
import ast
from typing import Any, Dict, List, Set, Optional

from rclpy.action import get_action_names_and_types # type: ignore[import]

from r2pilot_server.utils.constants import RosMsgTypes, RobotInfoKeys, DiscoveryConfig

class DiscoveryManager:
    """!
    @brief Clase gestora para explorar el grafo de ROS 2.
    @details Se inicializa con una referencia al nodo principal para poder consultar
             los tópicos y servicios activos.
    """
    def __init__(self, node: Any) -> None:
        """!
        @brief Constructor del gestor de descubrimiento.
        @param node Referencia al nodo principal (TiagoBridgeNode).
        """
        self.node = node
        self.logger = logging.getLogger("DiscoveryManager")
        
        self.joint_limits: List[Dict[str, Any]] = []
        self.dynamic_controllers: Dict[str, List[str]] = {} 
        self._visited_controllers: Set[str] = set()

    def parse_urdf(self, urdf_xml: str) -> None:
        """! 
        @brief Lee el XML del robot y extrae los límites de los motores reales. 
        @details Se almacenan en `self.joint_limits` para validar comandos de posición.
        @param urdf_xml String conteniendo la estructura completa del URDF.
        """
        if self.joint_limits: 
            return
        try:
            root = ET.fromstring(urdf_xml)
            limits = []
            for joint in root.findall('joint'):
                j_type = joint.get('type')
                j_name = joint.get('name')
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
            self.logger.info(f"[URDF] Parseo completado. {len(limits)} motores detectados.")
        except Exception as e:
            self.logger.error(f"[URDF] Error parseando el URDF del robot: {e}")

    def resolve_controller_joints_async(self, topic_name: str) -> None:
        """!
        @brief Lanza un hilo para identificar qué motores pertenecen a qué controlador.
        @param topic_name Nombre del controlador detectado.
        """
        if topic_name not in self._visited_controllers:
            self._visited_controllers.add(topic_name)
            threading.Thread(target=self._resolve_controller_joints_sync, args=(topic_name,), daemon=True).start()

    def _resolve_controller_joints_sync(self, topic_name: str) -> None:
        """! 
        @brief Función interna síncrona que ejecuta las llamadas a la terminal.
        @details Se utiliza para no bloquear el hilo principal del nodo mientras se resuelven los controladores.
        @param topic_name Nombre del controlador detectado.
        @return None. Actualiza `self.dynamic_controllers` si se detectan motores.
        """
        
        parts = topic_name.split('/')
        if len(parts) < 3: return
        
        controller_name = parts[1]
        joints_list = []
        target_phrases = ["String array value is:", "String values are:"]

        def extract_joints(output_text: str) -> list:
            """!
            @brief Extrae la lista de articulaciones del texto de salida de ros2 param get.
            @param output_text Texto crudo devuelto por el comando.
            @return Lista de nombres de articulaciones o lista vacía si no se pudo parsear
            """
            for phrase in target_phrases:
                if phrase in output_text:
                    list_str = output_text.split(phrase)[1].strip()
                    try:
                        return ast.literal_eval(list_str)
                    except Exception:
                        pass
            return []
        
        # Intentamos primero con el prefijo del controller_manager
        try:
            res = subprocess.check_output(
                ['ros2', 'param', 'get', '/controller_manager', f'{controller_name}.joints'],
                text=True, timeout=10.0, stderr=subprocess.DEVNULL 
            )
            joints_list = extract_joints(res)
        except Exception:
            pass

        # Si no encontramos nada, intentamos con el nombre directo del controlador    
        if not joints_list:
            try:
                res = subprocess.check_output(
                    ['ros2', 'param', 'get', f'/{controller_name}', 'joints'],
                    text=True, timeout=10.0, stderr=subprocess.DEVNULL 
                )
                joints_list = extract_joints(res)
            except Exception:
                pass

        # Si encontramos articulaciones, las almacenamos en el diccionario de controladores dinámicos        
        if joints_list:
            self.dynamic_controllers[topic_name] = joints_list
            self.logger.info(f"[DISCOVERY] Controlador: '{topic_name}' maneja {len(joints_list)} motores.")
        else:
            if topic_name in self._visited_controllers:
                self._visited_controllers.remove(topic_name)

    def get_teleop_topics(self) -> List[str]:
        """! 
        @brief Descubre qué tópicos en la red sirven para mover chasis o ruedas. 
        @details Filtra por tipos Twist o TwistStamped y aplica heurística de nombres.
        @return Lista de tópicos ordenada por prioridad heurística.
        """
        topics_and_types = self.node.get_topic_names_and_types()
        safe_teleop_topics: List[str] = []
        safe_base_keywords = ['cmd_vel', 'cmd_vel_unstamped', 'cmd_vel_stamped', 'teleop', 'velocity', 'twist', 'diff_drive', 'base_controller', 'mobile_base_controller', 'wheel', 'drive', 'movement']

        def _topic_priority(topic_name: str) -> int:
            """!
            @brief Asigna un índice de prioridad heurística a un tópico de teleoperación.
            @param topic_name Nombre del tópico a evaluar.
            @return Índice de prioridad (0 = más prioritario, mayor = menos prioritario).
            """ 
            lower_name = topic_name.lower()
            for index, keyword in enumerate(safe_base_keywords):
                if keyword in lower_name: return index
            return len(safe_base_keywords)

        # Filtramos los tópicos que sean de tipo Twist o TwistStamped y aplicamos heurística de nombres
        for name, types in topics_and_types:
            if RosMsgTypes.TWIST in types or RosMsgTypes.TWIST_STAMPED in types:
                safe_teleop_topics.append(name)

        safe_teleop_topics.sort(key=lambda topic: (_topic_priority(topic), len(topic), topic))
        return safe_teleop_topics

    def get_camera_topics(self) -> List[str]:
        """! 
        @brief Descubre tópicos de imagen de color crudo discriminando las capas profundas. 
        @details Filtra por tipos Image o CompressedImage y aplica heurística de nombres.
        @return Lista de tópicos ordenada por prioridad heurística.
        """
        topics_and_types = self.node.get_topic_names_and_types()
        camera_topics: List[str] = []
        exclude_keywords = ['disparity', 'mask', 'segmentation', 'semantic', 'instance', 'optical_flow', 'stereo']
        priority_keywords = ['camera', 'image_raw', 'rgb', 'color', 'front', 'main']

        def _camera_priority(topic_name: str) -> int:
            """!
            @brief Asigna un índice de prioridad heurística a un tópico de cámara.
            @param topic_name Nombre del tópico a evaluar.
            @return Índice de prioridad (0 = más prioritario, mayor = menos prioritario).
            """
            lower_name = topic_name.lower()
            for index, keyword in enumerate(priority_keywords):
                if keyword in lower_name: return index
            return len(priority_keywords)

        # Filtramos los tópicos que sean de tipo Image o CompressedImage y aplicamos heurística de nombres
        for topic_name, types in topics_and_types:
            if RosMsgTypes.IMAGE in types or RosMsgTypes.COMPRESSED_IMAGE in types:
                lower_name = topic_name.lower()
                if any(excl in lower_name for excl in exclude_keywords): continue
                camera_topics.append(topic_name)

        camera_topics.sort(key=lambda name: (_camera_priority(name), len(name), name))
        return camera_topics

    def get_available_sensors(self) -> List[Dict[str, str]]:
        """! 
        @brief Escanea la red de ROS 2 y genera un índice de sensores. 
        @details Filtra por tipos de mensaje de sensores y descarta tópicos irrelevantes.
        @return Lista de diccionarios con 'topic' y 'type' de cada sensor detectado.
        """
        topics_and_types = self.node.get_topic_names_and_types()
        sensors_list = []
        supported_types = {
            RosMsgTypes.LASER_SCAN: "LaserScan", RosMsgTypes.IMU: "Imu",
            RosMsgTypes.BATTERY: "BatteryState", RosMsgTypes.RANGE: "Range",
            RosMsgTypes.POINT_CLOUD2: "PointCloud2", RosMsgTypes.ODOMETRY: "Odometry",
            RosMsgTypes.NAV: "NavSatFix", RosMsgTypes.WRENCH: "Wrench",
            RosMsgTypes.TEMPERATURE: "Temperature"
        }
        exclude_keywords = ['throttle', 'filtered', 'depth/points']
        
        # Filtramos los tópicos que sean de tipo sensor y aplicamos heurística de nombres
        for topic_name, types in topics_and_types:
            topic_lower = topic_name.lower()
            if any(excl in topic_lower for excl in exclude_keywords): continue
            for ros_type, clean_name in supported_types.items():
                if ros_type in types:
                    sensors_list.append({"topic": topic_name, "type": clean_name})
                    break 
                    
        sensors_list.sort(key=lambda x: (x["type"], x["topic"]))
        return sensors_list

    def build_robot_info(self, latest_battery: Optional[float], latest_estop: Optional[bool], latest_charging: Optional[bool], current_joint_states: Dict[str, float]) -> Dict[str, Any]:
        """!
        @brief Recopila el inventario de topología de red completo e infiere la anatomía.
        @param latest_battery Nivel de batería (0-100).
        @param latest_estop Estado del botón de emergencia.
        @param latest_charging Estado de carga de corriente.
        @param current_joint_states Diccionario con las posiciones actuales de los motores.
        @return Diccionario estructurado del contrato R2Pilot.
        """
        hostname = socket.gethostname()
        domain_id = os.environ.get("ROS_DOMAIN_ID", None)
        
        has_base, has_manipulator, has_head, has_torso, has_gripper = False, False, False, False, False
        has_lidar, has_imu, has_odom, has_nav, has_moveit, has_ft_sensor, has_play_motion = False, False, False, False, False, False, False

        cameras_list: List[Dict[str, str]] = []  
        detected_camera_roots = set()
        topics_and_types = self.node.get_topic_names_and_types()

        for topic_name, types in topics_and_types:
            topic_lower = topic_name.lower()
            
            # Heurística de detección de capacidades basada en tipos de mensaje y palabras clave
            if RosMsgTypes.TWIST in types or RosMsgTypes.TWIST_STAMPED in types:
                if any(keyword in topic_lower for keyword in DiscoveryConfig.BASE_KEYWORDS): has_base = True
            
            if RosMsgTypes.CAMERA_INFO in types:
                hw_root = topic_name
                for suffix in DiscoveryConfig.CAMERA_CLEANUP_SUFFIXES:
                    hw_root = hw_root.replace(suffix, "")
                if hw_root not in detected_camera_roots:
                    detected_camera_roots.add(hw_root)
                    display_name = hw_root.split('/')[-1].replace('_', ' ').title()
                    if not display_name: display_name = DiscoveryConfig.DEFAULT_CAMERA_NAME
                    cameras_list.append({"name": display_name})
            
            if RosMsgTypes.LASER_SCAN in types:
                has_lidar = True
            elif RosMsgTypes.POINT_CLOUD2 in types:
                if not any(excl in topic_lower for excl in DiscoveryConfig.LIDAR_EXCLUDE_KEYWORDS): has_lidar = True
            
            if RosMsgTypes.JOINT_TRAJ in types:
                if any(keyword in topic_lower for keyword in DiscoveryConfig.ARM_KEYWORDS): has_manipulator = True
                if any(keyword in topic_lower for keyword in DiscoveryConfig.GRIPPER_KEYWORDS): has_gripper = True
            
            if RosMsgTypes.IMU in types: has_imu = True
            if RosMsgTypes.ODOMETRY in types: has_odom = True
            if RosMsgTypes.OCCUPANCY_GRID in types: has_nav = True
            if RosMsgTypes.MOVEIT_PLANNING_SCENE in types: has_moveit = True
            if RosMsgTypes.WRENCH in types:
                if any(keyword in topic_lower for keyword in ['ft_sensor', 'wrench', 'force', 'torque']): has_ft_sensor = True
            
            if any(keyword in topic_lower for keyword in ['torso', 'lift', 'spine', 'elevator']): has_torso = True
            if any(keyword in topic_lower for keyword in ['head', 'neck', 'pan_tilt', 'ptu']): has_head = True
            if 'play_motion' in topic_lower: has_play_motion = True

        #Generamos la lista de tópicos de teleoperación y cámaras según las capacidades detectadas
        teleop_topics = self.get_teleop_topics() if has_base else []
        camera_topics = self.get_camera_topics() if cameras_list else []

        # Generamos la lista de motores activos según los controladores dinámicos detectados
        motores_activos = set()
        for joints_list in self.dynamic_controllers.values():
            motores_activos.update(joints_list)

        updated_joint_limits = []
        for limit in self.joint_limits:
            limit_copy = limit.copy()
            limit_copy["current_value"] = current_joint_states.get(limit["name"], 0.0)
            limit_copy["is_actuated"] = limit["name"] in motores_activos
            updated_joint_limits.append(limit_copy)
        
        # Construimos el diccionario final de capacidades del robot según el contrato R2Pilot
        return {
            RobotInfoKeys.IDENTITY: {"hostname": hostname, "domain_id": domain_id},
            RobotInfoKeys.STATUS: {"battery_pct": latest_battery, "e_stop_active": latest_estop, "is_charging": latest_charging},
            RobotInfoKeys.CAPABILITIES: {
                RobotInfoKeys.HAS_BASE: has_base, RobotInfoKeys.CAMERAS: cameras_list,
                RobotInfoKeys.TELEOP_TOPICS: teleop_topics, RobotInfoKeys.CAMERA_TOPICS: camera_topics,
                RobotInfoKeys.HAS_MANIPULATOR: has_manipulator, RobotInfoKeys.HAS_HEAD: has_head,
                RobotInfoKeys.HAS_TORSO: has_torso, RobotInfoKeys.HAS_GRIPPER: has_gripper,
                RobotInfoKeys.HAS_IMU: has_imu, RobotInfoKeys.HAS_ODOMETRY: has_odom,
                RobotInfoKeys.HAS_LIDAR: has_lidar, RobotInfoKeys.HAS_NAV: has_nav,
                RobotInfoKeys.HAS_MOVEIT: has_moveit, RobotInfoKeys.HAS_FT_SENSOR: has_ft_sensor,
                RobotInfoKeys.HAS_PLAY_MOTION: has_play_motion, RobotInfoKeys.CONTROLABLE_JOINTS: updated_joint_limits
            }
        }

    # =========================================================================
    # INSPECCIÓN DE RED GLOBAL (Estilo ros2cli)
    # =========================================================================
    
    def get_all_topics(self) -> Dict[str, List[str]]:
        """! 
        @brief Devuelve un diccionario nativo con todos los topics y sus tipos de variable. 
        @return Diccionario {topic_name: [type1, type2, ...]} de todos los tópicos activos.
        """
        return {name: types for name, types in self.node.get_topic_names_and_types()}

    def get_all_services(self) -> Dict[str, List[str]]:
        """! 
        @brief Devuelve un diccionario nativo con todos los servicios y sus firmas. 
        @return Diccionario {service_name: [type1, type2, ...]} de todos los servicios activos.
        """
        return {name: types for name, types in self.node.get_service_names_and_types()}

    def get_all_actions(self) -> Dict[str, List[str]]:
        """! 
        @brief Descubre todas las interfaces de tipo Action encolables expuestas en la red. 
        @return Diccionario {action_name: [type1, type2, ...]} de todas las acciones activas.
        """
        try:
            return {name: types for name, types in get_action_names_and_types(self.node)}
        except Exception as e:
            self.logger.warning(f"Error obteniendo actions: {e}")
            return {}