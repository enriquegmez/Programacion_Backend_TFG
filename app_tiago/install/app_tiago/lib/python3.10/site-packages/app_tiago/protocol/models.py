"""
models.py
Representación interna del protocolo TIAGO Robot.
Basado estrictamente en la especificación JSON Schema del proyecto.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Union
import time

# --- ESTRUCTURAS DE CABECERA ---

@dataclass
class MessageHeader:
    msg_id: int
    type: str          # Debe coincidir con constants.MsgType
    session_id: str
    timestamp: float = field(default_factory=time.time)

# --- PAYLOADS DE PETICIÓN (Requests) ---

@dataclass
class CommandReqPayload:
    action: str        # connect, disconnect, etc.
    param1: Optional[str] = None
    param2: Optional[str] = None
    param3: Optional[bool] = None

@dataclass
class QueryReqPayload:
    resource_type: str # TOPICS, SENSORS, ACTIONS, ROBOT_INFO

@dataclass
class ActionReqPayload:
    type: str          # JOINT, HOME, EXEC_ACTION
    target: str

# --- ¡NUEVO! ---
@dataclass
class StopActionReqPayload:
    type: str          # Debe coincidir con la acción actual (ej. EXEC_ACTION)
    target: str        # Debe coincidir con el target actual (ej. saludar)

@dataclass
class ControlModeReqPayload:
    event: str         # START, STOP
    type: str          # TELEOP, JOINT
    topic: Optional[str] = "cmd_vel"

@dataclass
class ControlData:
    v: Optional[float] = 0.0
    w: Optional[float] = 0.0
    joint_name: Optional[str] = None
    joint_value: Optional[float] = None

@dataclass
class ControlReqPayload:
    data: ControlData

@dataclass
class StreamReqPayload:
    resource: str
    topic: Optional[str] = None
    quality_level: Optional[str] = None

@dataclass
class StopStreamReqPayload:
    resource: str
    topic: Optional[str] = None # ¡NUEVO! Para poder apagar un sensor específico sin apagar

@dataclass
class AsyncNotifyPayload:
    type: str
    details: str
    severity: Optional[str] = None

@dataclass
class ProtocolErrorPayload:
    error_code: int
    description: str

@dataclass
class EmptyPayload:
    """Usado para PING_REQ y ACK, que según el Schema no tienen properties"""
    pass

# --- PAYLOADS DE RESPUESTA (Responses) ---

@dataclass
class BaseResponsePayload:
    """Campos base que comparten todas las respuestas (RESP)"""
    success: bool
    code: int
    resp_type: str     # Ej: COMMAND_RESP, QUERY_RESP
    details: Optional[str] = None
    resp_data: Dict[str, Any] = field(default_factory=dict)

@dataclass
class QueryRespPayload(BaseResponsePayload):
    """
    Respuesta específica para QUERY_REQ.
    Puede contener una lista de strings (ej. topics), un diccionario complejo (ROBOT_INFO),
    o una lista de diccionarios (MENÚ DE SENSORES).
    """
    # ¡NUEVO! Añadimos List[Dict[str, Any]] a las opciones permitidas
    data: Optional[Union[List[str], Dict[str, Any], List[Dict[str, Any]]]] = None

@dataclass
class ActionFeedbackPayload(BaseResponsePayload):
    """Respuesta específica para ACTION_REQ y STOP_ACTION_REQ"""
    done_exec: Optional[bool] = None
    progress: Optional[int] = None
    status: Optional[str] = None

@dataclass
class StreamRespPayload(BaseResponsePayload):
    """Respuesta específica para STREAM_REQ"""
    stream_data: Optional[Dict[str, Any]] = None
    stream_url: Optional[str] = None

@dataclass
class GenericRespPayload(BaseResponsePayload):
    """Para COMMAND_RESP, CONTROL_MODE_RESP, CONTROL_RESP, STOP_STREAM_RESP (No añaden campos)"""
    pass

# --- CONTENEDOR PRINCIPAL ---

@dataclass
class RobotMessage:
    """
    Clase maestra que representa cualquier mensaje del protocolo.
    """
    header: MessageHeader
    payload: Union[
        # Requests
        CommandReqPayload, 
        QueryReqPayload,
        ActionReqPayload,
        StopActionReqPayload,
        ControlModeReqPayload, 
        ControlReqPayload, 
        StreamReqPayload,
        StopStreamReqPayload,
        AsyncNotifyPayload,
        ProtocolErrorPayload,
        EmptyPayload,
        # Responses
        QueryRespPayload,
        ActionFeedbackPayload,
        StreamRespPayload,
        GenericRespPayload
    ]