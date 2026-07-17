## @file constants.py
#  @brief Definición de constantes globales y enumeraciones.
#  @details Centraliza los literales mágicos, límites de seguridad y estructuras 
#           del JSON Schema para asegurar consistencia entre el backend de ROS 2 
#           y el frontend (aplicación móvil).
#  @author Enrique Gómez
#  @date 2026

# =============================================================================
# 1. CONFIGURACIÓN DEL SERVIDOR WEB Y SESIÓN
# =============================================================================

SERVER_IP = "0.0.0.0"
SERVER_PORT = 8765

class SessionTimeout:
    """! @brief Tiempos de espera (en segundos) para la gestión de la conexión."""
    PING_TIMEOUT = 3.0                    # Tiempo máximo sin recibir PING antes de abortar
    CONTROL_RESP_INTERVAL = 0.5           # Tasa de envío de respuestas de control
    AUTO_DISCOVERY_TOPICS_INTERVAL = 2.0  # Periodo de escaneo de nuevos tópicos en ROS 2

# =============================================================================
# 2. CAPA DE PROTOCOLO DE RED (JSON SCHEMA)
# =============================================================================

class MsgType:
    """! @brief Tipos de mensaje principales (Campo 'type' en el Header)."""
    COMMAND_REQ = "COMMAND_REQ"
    RESP = "RESP"  
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

class RespType:
    """! @brief Subtipos de respuesta (Campo 'resp_type' en el Payload cuando Header es RESP)."""
    COMMAND_RESP = "COMMAND_RESP"
    QUERY_RESP = "QUERY_RESP"
    ACTION_FEEDBACK = "ACTION_FEEDBACK"
    STOP_ACTION_FEEDBACK = "STOP_ACTION_FEEDBACK"
    CONTROL_MODE_RESP = "CONTROL_MODE_RESP"
    CONTROL_RESP = "CONTROL_RESP"
    STREAM_RESP = "STREAM_RESP"
    STOP_STREAM_RESP = "STOP_STREAM_RESP"

class StatusCode:
    """! @brief Códigos de estado HTTP-like para estandarizar errores."""
    OK = 200
    BAD_REQUEST = 400        # Fallo de formato o JSON malformado
    FORBIDDEN = 403          # Acción bloqueada por seguridad
    NOT_FOUND = 404          # Recurso no encontrado
    NOT_ALLOWED = 405        # Estado de la FSM incorrecto para esa petición
    INTERNAL_ERROR = 500     # Excepción en el core de Python o puente ROS 2

# =============================================================================
# 3. SEMÁNTICA DE ACCIONES Y RECURSOS
# =============================================================================

class Action:
    """! @brief Comandos de sistema (CommandReq payload 'action')."""
    CONNECT = "connect"
    DISCONNECT = "disconnect"
    CHANGE_VARS = "change_vars"   # Configuración de variables de red (DDS)
    REBOOT = "reboot"            # Reinicio de la máquina anfitriona (PC)
    SHUTDOWN = "shutdown"        # Apagado controlado de la máquina anfitriona
    END = "end"

class ActionType:
    """! @brief Clasificación de las acciones físicas de ROS 2."""
    EXEC_ACTION = "EXEC_ACTION"  # Animaciones complejas (ej. play_motion)


class Resource:
    """! @brief Identificadores de recursos solicitables mediante QUERY_REQ o STREAM_REQ."""
    HOST_INFO = "HOST_INFO"      # Telemetría del PC (CPU, RAM, Temperaturas)
    ROBOT_INFO = "ROBOT_INFO"    # Capacidades inferidas del robot
    TELEOP = "TELEOP"
    CAMERAS = "CAMERAS"
    CAMERA = "CAMERA"
    TOPICS = "TOPICS"
    SERVICES = "SERVICES"
    ACTIONS = "ACTIONS"
    MOVEMENTS = "MOVEMENTS"
    SENSORS = "SENSORS"

class ControlEvent:
    """! @brief Eventos de ciclo de vida del joystick / control."""
    START = "START"
    STOP = "STOP"

class ControlType:
    """! @brief Naturaleza del actuador a controlar."""
    TELEOP = "TELEOP"
    JOINT = "JOINT"

# =============================================================================
# 4. MÁQUINA DE ESTADOS FINITOS (FSM)
# =============================================================================

class ServerState:
    """! @brief Estados principales del gestor de red."""
    IDLE = "IDLE"
    CONEXION_BACKEND = "CONEXION_BACKEND"
    SESION_INICIADA = "SESION_INICIADA"

class MovementState:
    """! @brief Sub-estados concurrentes (Lógica de actuadores)."""
    IDLE = "IDLE"                            
    RECIBIENDO_INFO = "RECIBIENDO_INFO"      # Activo tras un ControlModeReq[START]
    EJECUTANDO_ACCION = "EJECUTANDO_ACCION"  # Activo durante una acción prologada

class MonitorState:
    """! @brief Sub-estados concurrentes (Lógica de telemetría y vídeo)."""
    IDLE = "IDLE"                            
    ENVIANDO_STREAM = "ENVIANDO_STREAM"      # Activo durante la emisión de sensores o cámaras

# =============================================================================
# 5. ROBÓTICA Y ROS 2: CONFIGURACIÓN Y SEGURIDAD
# =============================================================================

class RobotLimits:
    """! @brief Restricciones físicas impuestas por el SafetyFilterNode."""
    MAX_LINEAR_VEL = 0.5    # [m/s] Límite alineado con validación de JSON Schema
    MAX_ANGULAR_VEL = 1.0   # [rad/s] Límite alineado con validación de JSON Schema
    SAFE_DIST = 0.5         # [m] Distancia mínima al obstáculo antes de frenar

class TeleopConfig:
    """! @brief Parámetros del Watchdog de teleoperación."""
    TIMEOUT = 0.5           # [s] Límite sin comandos antes de activar el freno de emergencia

class RosMsgTypes:
    """! @brief Diccionario de tipos de mensajes de ROS 2 para inferencia dinámica."""
    IMAGE = "sensor_msgs/msg/Image"                  
    COMPRESSED_IMAGE = "sensor_msgs/msg/CompressedImage" 
    LASER_SCAN = "sensor_msgs/msg/LaserScan"         
    BATTERY = "sensor_msgs/msg/BatteryState"         
    TWIST = "geometry_msgs/msg/Twist"                
    JOINT_TRAJ = "trajectory_msgs/msg/JointTrajectory" 
    OCCUPANCY_GRID = "nav_msgs/msg/OccupancyGrid"    
    TWIST_STAMPED = "geometry_msgs/msg/TwistStamped"
    CAMERA_INFO = "sensor_msgs/msg/CameraInfo"
    POINT_CLOUD2 = "sensor_msgs/msg/PointCloud2"
    IMU = "sensor_msgs/msg/Imu"                      
    ODOMETRY = "nav_msgs/msg/Odometry"               
    JOINT_STATE = "sensor_msgs/msg/JointState"       
    MOVEIT_PLANNING_SCENE = "moveit_msgs/msg/PlanningScene"
    RANGE = "sensor_msgs/msg/Range"                  
    NAV = 'sensor_msgs/msg/NavSatFix'
    WRENCH = 'geometry_msgs/msg/WrenchStamped'
    TEMPERATURE = 'sensor_msgs/msg/Temperature'
    FLOAT32 = 'std_msgs/msg/Float32'
    BOOL = 'std_msgs/msg/Bool'

class DiscoveryConfig:
    """! @brief Palabras clave para auto-descubrimiento de hardware."""
    BASE_KEYWORDS = ['cmd_vel', 'base', 'diff']
    ARM_KEYWORDS = ['arm', 'manipulator', 'ur5', 'kinova']
    GRIPPER_KEYWORDS = ['gripper', 'hand', 'finger']
    LIDAR_EXCLUDE_KEYWORDS = ['depth', 'voxel']
    
    CAMERA_CLEANUP_SUFFIXES = ["/camera_info", "/rgb", "/depth", "/color", "/infra1", "/infra2"]
    DEFAULT_CAMERA_NAME = "Cámara Principal"
    
    CAMERA_INFO_STR = "camera_info"
    IMAGE_RAW_STR = "image_raw"

    MOVEIT_NODES = ['move_group']

# =============================================================================
# 6. ESTRUCTURAS DE RETORNO Y TELEMETRÍA FRONTEND
# =============================================================================

class StreamConfig:
    """! @brief Perfiles de resolución y compresión para web_video_server."""
    CAMERA_PROFILES = {
        "low": "&width=320&height=240&quality=30",
        "medium": "&width=640&height=480&quality=60",
        "high": "&width=1024&height=768&quality=90"
    }

class RobotInfoKeys:
    """! @brief Claves exactas del JSON entregado en Query[ROBOT_INFO]."""
    IDENTITY = "identity"
    STATUS = "status"
    CAPABILITIES = "capabilities"
    
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
    CONTROLABLE_JOINTS = "controlable_joints"

class TelemetryKeys:
    """! @brief Claves exactas del JSON entregado en Query[HOST_INFO]."""
    CPU_PCT = "cpu_pct"
    RAM_USED_GB = "ram_used_gb"
    RAM_TOTAL_GB = "ram_total_gb"
    RAM_PCT = "ram_pct"
    TEMP_C = "temp_c"
    ROS_DISTRO = "ros_distro"
    ROS_DOMAIN_ID = "ros_domain_id"
    CURRENT_DDS = "current_dds"
    AVAILABLE_DDS = "available_dds"
    USE_DISCOVERY = "use_discovery"