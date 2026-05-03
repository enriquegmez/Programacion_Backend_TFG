"""
constants.py
Definición de todas las constantes del protocolo para asegurar la integridad
entre el backend y el frontend.
"""

# --- CONFIGURACIÓN DEL SERVIDOR ---
# "0.0.0.0" permite que el servidor escuche en todas las interfaces de red del robot
SERVER_IP = "0.0.0.0"
SERVER_PORT = 8765

# --- TIPOS DE MENSAJE (HEADER) ---
class MsgType:
    RESP = "RESP"
    COMMAND_REQ = "COMMAND_REQ"
    COMMAND_RESP = "COMMAND_RESP"
    
    QUERY_REQ = "QUERY_REQ"
    QUERY_RESP = "QUERY_RESP"
    
    ACTION_REQ = "ACTION_REQ"
    ACTION_FEEDBACK = "ACTION_FEEDBACK"
    STOP_ACTION_REQ = "STOP_ACTION_REQ"
    STOP_ACTION_RESP = "STOP_ACTION_RESP"
    
    CONTROL_MODE_REQ = "CONTROL_MODE_REQ"
    CONTROL_MODE_RESP = "CONTROL_MODE_RESP"
    CONTROL_REQ = "CONTROL_REQ"
    CONTROL_RESP = "CONTROL_RESP"
    
    STREAM_REQ = "STREAM_REQ"
    STREAM_RESP = "STREAM_RESP"
    STOP_STREAM_REQ = "STOP_STREAM_REQ"
    STOP_STREAM_RESP = "STOP_STREAM_RESP"
    
    ASYNC_NOTIFY = "ASYNC_NOTIFY"
    ASYNC_NOTIFY_RESP = "ASYNC_NOTIFY_RESP"
    
    PING_REQ = "PING_REQ"
    PING_RESP = "PING_RESP"
    
    PROTOCOL_ERROR = "PROTOCOL_ERROR"
    PROTOCOL_ERROR_RESP = "PROTOCOL_ERROR_RESP"

# --- ACCIONES ESPECÍFICAS (Para CommandReq) ---
class Action:
    CONNECT = "connect"
    DISCONNECT = "disconnect"
    CHANGE_VAR = "change_variables"
    GET_HISTORY = "get_sensor_history"
    SUBSCRIBE = "subscribe_topic"
    SSH_CMD = "ssh_command"

# --- RECURSOS (Para QueryReq y StreamReq) ---
class Resource:
    ROBOT_INFO = "robot_info"
    TOPICS = "topics"
    SENSORS = "sensors"
    ACTIONS = "actions"
    MOVEMENTS = "movements"
    CAMERA = "camera"
    STATUS = "status"

# --- CÓDIGOS DE ESTADO / ERROR ---
class StatusCode:
    OK = 200
    BAD_REQUEST = 400         # Fallo de formato o JSON malformado
    FORBIDDEN = 403           # Acción no permitida (ej: batería baja)
    NOT_ALLOWED = 405         # Estado incorrecto para esa petición
    INTERNAL_ERROR = 500      # Fallo en el backend o en ROS 2

# --- EVENTOS DE CONTROL ---
class ControlEvent:
    START = "start"
    STOP = "stop"