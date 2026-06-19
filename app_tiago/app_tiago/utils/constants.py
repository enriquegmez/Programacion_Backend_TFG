"""
constants.py
Definición de todas las constantes del protocolo para asegurar la integridad
entre el backend y el frontend, siguiendo estrictamente el JSON Schema.
"""

# --- CONFIGURACIÓN DEL SERVIDOR ---
SERVER_IP = "0.0.0.0"
SERVER_PORT = 8765

# --- TIPOS DE MENSAJE (HEADER type) ---
# Deben coincidir exactamente con el enum del header en el JSON Schema
class MsgType:
    COMMAND_REQ = "COMMAND_REQ"
    RESP = "RESP"  # Todas las respuestas usan este tipo en el header
    QUERY_REQ = "QUERY_REQ"
    ACTION_REQ = "ACTION_REQ"
    STOP_ACTION_REQ = "STOP_ACTION_REQ"
    CONTROL_MODE_REQ = "CONTROL_MODE_REQ"
    CONTROL_REQ = "CONTROL_REQ"
    STREAM_REQ = "STREAM_REQ"
    STOP_STREAM_REQ = "STOP_STREAM_REQ"
    PING_REQ = "PING_REQ"
    ASYNC_NOTIFY = "ASYNC_NOTIFY"
    PROTOCOL_ERROR = "PROTOCOL_ERROR"
    ACK = "ACK"

# --- TIPOS DE RESPUESTA (PAYLOAD resp_type) ---
# Cuando el header es "RESP", este campo define qué respuesta es
class RespType:
    COMMAND_RESP = "COMMAND_RESP"
    QUERY_RESP = "QUERY_RESP"
    ACTION_FEEDBACK = "ACTION_FEEDBACK"
    STOP_ACTION_FEEDBACK = "STOP_ACTION_FEEDBACK"
    CONTROL_MODE_RESP = "CONTROL_MODE_RESP"
    CONTROL_RESP = "CONTROL_RESP"
    STREAM_RESP = "STREAM_RESP"
    STOP_STREAM_RESP = "STOP_STREAM_RESP"

# --- ACCIONES ESPECÍFICAS (CommandReq payload 'action') ---
class Action:
    CONNECT = "connect"
    DISCONNECT = "disconnect"
    CHANGE_VAR = "change_vars"
    END = "end"
    GET_HISTORY = "get_history"
    SSH_CMD = "ssh"

# --- TIPOS DE ACCIÓN (ActionReq payload 'type') ---
class ActionType:
    EXEC_ACTION = "exec_action"  # Para lanzar animaciones de play_motion
    JOINT = "joint"              # (Opcional a futuro) Mover articulaciones
    HOME = "home"                # (Opcional a futuro) Posición de reposo

# --- RECURSOS (QueryReq resource_type) ---
class Resource:
    ROBOT_INFO = "ROBOT_INFO"
    TELEOP = "TELEOP"
    CAMERAS = "CAMERAS"
    TOPICS = "TOPICS"
    SENSORS = "SENSORS"
    ACTIONS = "ACTIONS"

# --- EVENTOS Y TIPOS DE CONTROL (ControlModeReq payload) ---
class ControlEvent:
    START = "START"
    STOP = "STOP"

class ControlType:
    TELEOP = "TELEOP"
    JOINT = "JOINT"

# --- CÓDIGOS DE ESTADO / ERROR ---
class StatusCode:
    OK = 200
    BAD_REQUEST = 400        # Fallo de formato o JSON malformado
    FORBIDDEN = 403          # Acción no permitida (ej: batería baja)
    NOT_ALLOWED = 405        # Estado incorrecto para esa petición
    INTERNAL_ERROR = 500     # Fallo en el backend o en ROS 2
    NOT_FOUND = 404

# --- ESTADOS DEL SERVIDOR (Para state_machine.py) ---
class ServerState:
    IDLE = "IDLE"
    CONEXION_BACKEND = "CONEXION_BACKEND"
    SESION_INICIADA = "SESION_INICIADA"

# Sub-estados de la región concurrente: Movimiento (Servidor)
class MovementState:
    IDLE = "IDLE"                            # Estado base (punto negro en el diagrama)
    RECIBIENDO_INFO = "RECIBIENDO_INFO"      # Activo tras ControlModeReq[start]
    EJECUTANDO_ACCION = "EJECUTANDO_ACCION"  # Activo tras ActionReq

# Sub-estados de la región concurrente: Monitorización (Servidor)
class MonitorState:
    IDLE = "IDLE"                            # Estado base (punto negro)
    ENVIANDO_STREAM = "ENVIANDO_STREAM"      # Activo tras StreamReq

# --- CONFIGURACIÓN DE ROS 2 (Interfaz Robótica) ---
class RosTopics:
    CMD_VEL = "/mobile_base_controller/cmd_vel" # Tópico estándar de Tiago
    BATTERY = "/battery_status"

# --- SERVIDORES DE ACCIÓN DE ROS 2 ---
class RosActions:
    # Dependiendo de tu versión de Tiago, puede ser "/play_motion" o "/play_motion2"
    PLAY_MOTION = "/play_motion"

# --- LÍMITES FÍSICOS DEL ROBOT (Seguridad / Validator) ---
class RobotLimits:
    MAX_LINEAR_VEL = 2.0    # Ajustado al JSON Schema (v: minimum -2.0, maximum 2.0)
    MAX_ANGULAR_VEL = 1.5   # Ajustado al JSON Schema (w: minimum -1.5, maximum 1.5)

# --- CONFIGURACIÓN DE TELEOPERACIÓN (Watchdog) ---
class TeleopConfig:
    TIMEOUT = 0.5  # Segundos sin recibir CONTROL_REQ antes de parar el robot por seguridad

    # --- TIPOS DE MENSAJES DE ROS 2 (Para la Autodetección) ---
class RosMsgTypes:
    IMAGE = "sensor_msgs/msg/Image"                  # Cámaras (crudo)
    COMPRESSED_IMAGE = "sensor_msgs/msg/CompressedImage" # Cámaras comprimidas
    LASER_SCAN = "sensor_msgs/msg/LaserScan"         # Escáneres láser / LiDAR
    BATTERY = "sensor_msgs/msg/BatteryState"         # Batería estándar de ROS 2
    TWIST = "geometry_msgs/msg/Twist"                # Base Móvil (Ruedas)
    JOINT_TRAJ = "trajectory_msgs/msg/JointTrajectory" # Brazos / Manipuladores
    OCCUPANCY_GRID = "nav_msgs/msg/OccupancyGrid"    # Mapas de navegación (Nav2)
    TWIST_STAMPED = "geometry_msgs/msg/TwistStamped"
    CAMERA_INFO = "sensor_msgs/msg/CameraInfo"
    POINT_CLOUD2 = "sensor_msgs/msg/PointCloud2"
    IMU = "sensor_msgs/msg/Imu"                      # Sensor inercial
    ODOMETRY = "nav_msgs/msg/Odometry"               # Odometría del robot
    JOINT_STATE = "sensor_msgs/msg/JointState"       # Estado de articulaciones
    MOVEIT_PLANNING_SCENE = "moveit_msgs/msg/PlanningScene"

# --- CLAVES DEL DICCIONARIO JSON (Para el Frontend) ---
# Usamos constantes para no equivocarnos al teclear las keys del JSON de respuesta
class RobotInfoKeys:
    IDENTITY = "identity"
    STATUS = "status"
    CAPABILITIES = "capabilities"
    
    # Sub-claves de capacidades
    HAS_BASE = "has_base"
    CAMERAS = "cameras"
    HAS_MANIPULATOR = "has_manipulator"
    HAS_HEAD = "has_head"
    HAS_TORSO = "has_torso"
    HAS_LIDAR = "has_lidar"
    HAS_NAV = "has_nav"
    HAS_MOVEIT = "has_moveit"
    HAS_GRIPPER = "has_gripper"
    HAS_ODOMETRY = "has_odometry"
    HAS_IMU = "has_imu"
    HAS_FT_SENSOR = "has_ft_sensor"
    HAS_DIAGNOSTICS = "has_diagnostics"
    HAS_CHARGE_SENSOR = "has_charge_sensor"
    CAMERA_TOPICS = "camera_topics"
    TELEOP_TOPICS = "teleop_topics"
    HAS_PLAY_MOTION = "has_play_motion"

    # --- CONFIGURACIÓN DEL AUTO-DESCUBRIMIENTO (Heurísticas) ---
class DiscoveryConfig:
    # Palabras clave para inferir hardware
    BASE_KEYWORDS = ['cmd_vel', 'base', 'diff']
    ARM_KEYWORDS = ['arm', 'manipulator', 'ur5', 'kinova']
    GRIPPER_KEYWORDS = ['gripper', 'hand', 'finger']
    LIDAR_EXCLUDE_KEYWORDS = ['depth', 'voxel']
    
    # Procesamiento de nombres de cámara
    CAMERA_CLEANUP_SUFFIXES = ["/camera_info", "/rgb", "/depth", "/color", "/infra1", "/infra2"]
    DEFAULT_CAMERA_NAME = "Cámara Principal"
    
    # Reemplazo para obtener el topic de vídeo
    CAMERA_INFO_STR = "camera_info"
    IMAGE_RAW_STR = "image_raw"

    # Módulos complejos
    MOVEIT_NODES = ['move_group'] # Nodo principal de MoveIt