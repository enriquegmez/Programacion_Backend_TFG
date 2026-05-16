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

# --- RECURSOS (QueryReq resource_type) ---
class Resource:
    ROBOT_INFO = "ROBOT_INFO"
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

# --- LÍMITES FÍSICOS DEL ROBOT (Seguridad / Validator) ---
class RobotLimits:
    MAX_LINEAR_VEL = 2.0    # Ajustado al JSON Schema (v: minimum -2.0, maximum 2.0)
    MAX_ANGULAR_VEL = 1.5   # Ajustado al JSON Schema (w: minimum -1.5, maximum 1.5)

# --- CONFIGURACIÓN DE TELEOPERACIÓN (Watchdog) ---
class TeleopConfig:
    TIMEOUT = 0.5  # Segundos sin recibir CONTROL_REQ antes de parar el robot por seguridad