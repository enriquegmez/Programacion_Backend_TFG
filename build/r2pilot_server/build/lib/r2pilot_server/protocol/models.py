## @file models.py
#  @brief Representación interna del protocolo R2Pilot (Clases de Dominio).
#  @details Contiene la estructura de datos orientada a objetos (Dataclasses) del servidor.
#           Está diseñado para coincidir 1:1 de forma estricta con las reglas de 
#           protocol_schema.json, garantizando la compatibilidad con el cliente móvil.
#  @author Enrique Gómez
#  @date 2026

import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Union

# =========================================================================
# ESTRUCTURAS DE SENSORES (DTOs para telemetría)
# =========================================================================

@dataclass
class Vec3:
    """!
    @brief Representación de un vector 3D.
    """ 
    x: float; y: float; z: float

@dataclass
class Quat:
    """!
    @brief Representación de un cuaternión.
    """
    x: float; y: float; z: float; w: float

@dataclass
class Point2D:
    """!
    @brief Representación de un punto en el plano 2D.
    """
    x: float; y: float

@dataclass
class LaserData:
    """!
    @brief Representación de los datos del sensor lidar.
    """
    angle_min: float; angle_max: float; angle_increment: float
    range_min: float; range_max: float; ranges: List[float]

@dataclass
class ImuData:
    """!
    @brief Representación de los datos del sensor IMU.
    """
    orientation: Quat; angular_velocity: Vec3; linear_acceleration: Vec3

@dataclass
class BatteryData:
    """!
    @brief Representación de los datos del sensor de batería.
    """
    voltage: float; percentage: float; power_supply_status: int

@dataclass
class RangeData:
    """!
    @brief Representación de los datos del sensor de rango.
    """
    range: float; min_range: float; max_range: float; field_of_view: float

@dataclass
class PointCloudData:
    """!
    @brief Representación de los datos del nube de puntos.
    """
    width: int; height: int; is_dense: bool; note: str

@dataclass
class OdometryData:
    """!
    @brief Representación de los datos de odometría.
    """
    position: Point2D; linear_velocity: float; angular_velocity: float

@dataclass
class NavSatData:
    """!
    @brief Representación de los datos del sistema de navegación satelital.
    """
    latitude: float; longitude: float; altitude: float; status: int

@dataclass
class WrenchData:
    """!
    @brief Representación de los datos del sensor de fuerza y par.
    """
    force: Vec3; torque: Vec3

@dataclass
class TempData:
    """!
    @brief Representación de los datos del sensor de temperatura.
    """
    temperature: float

@dataclass
class SensorEnvelope:
    """! @brief Contenedor maestro que engloba a todos los sensores."""
    topic: str
    type: str
    data: Any

# =========================================================================
# ESTRUCTURAS DE CABECERA (Metadatos Universales)
# =========================================================================

@dataclass
class MessageHeader:
    """!
    @brief Cabecera de enrutamiento obligatoria en todos los mensajes.
    """
    msg_id: int
    type: str          # Ej: COMMAND_REQ, QUERY_REQ, RESP
    session_id: str
    timestamp: float = field(default_factory=time.time)

# =========================================================================
# PAYLOADS DE PETICIÓN (Requests del cliente al servidor)
# =========================================================================

@dataclass
class CommandReqPayload:
    """!
    @brief Tipo de mensaje para la realización de comandos por parte del servidor.
    """
    action: str        # connect, disconnect, change_vars, end, reboot, shutdown
    param1: Optional[str] = None
    param2: Optional[str] = None
    param3: Optional[bool] = None

@dataclass
class QueryReqPayload:
    """!
    @brief Tipo de mensaje para solicitudes de información del sistema.
    """
    resource_type: str # TOPICS, SENSORS, ACTIONS, MOVEMENTS, ROBOT_INFO, HOST_INFO

@dataclass
class ActionReqPayload:
    """!
    @brief Tipo de mensaje para lanzar tareas prolongadas en el robot (Action Servers de ROS 2).
    """
    type: str          # EXEC_ACTION
    target: str

@dataclass
class StopActionReqPayload:
    """!
    @brief Tipo de mensaje para cancelar una tarea prolongada en ejecución.
    """
    type: str          # EXEC_ACTION
    target: str

@dataclass
class ControlModeReqPayload:
    """!
    @brief Tipo de mensaje para iniciar o detener un modo de control (Teleoperación o Control de Articulaciones).
    """
    event: str         # START, STOP
    type: str          # TELEOP, JOINT
    topic: Optional[str] = "cmd_vel"

@dataclass
class ControlData:
    """!
    @brief Contenedor interno de variables cinemáticas para teleoperación o control de articulaciones.
    """
    v: Optional[float] = 0.0
    w: Optional[float] = 0.0
    joint_name: Optional[str] = None
    joint_value: Optional[float] = None

@dataclass
class ControlReqPayload:
    """!
    @brief Tipo de mensaje para enviar comandos de control al robot.
    """
    data: ControlData

@dataclass
class StreamReqPayload:
    """!
    @brief Tipo de mensaje para solicitar la visualización de datos en tiempo real.
    """
    resource: str      # CAMERA, SENSORS
    topic: Optional[str] = None
    quality_level: Optional[str] = None

@dataclass
class StopStreamReqPayload:
    """!
    @brief Tipo de mensaje para detener explícitamente un flujo de datos concreto.
    """
    resource: str      # CAMERA, SENSORS
    topic: Optional[str] = None

@dataclass
class AsyncNotifyPayload:
    """!
    @brief Tipo de mensaje para notificaciones Push originadas por el servidor.
    """
    type: str          # session_id, ALERT_BATTERY, EMERGENCY_STOP, CONFLICT_NOTIFY
    details: str
    severity: Optional[str] = None

@dataclass
class ProtocolErrorPayload:
    """!
    @brief Tipo de mensaje para reportar fallos a nivel de red o JSON Schema.
    """
    error_code: int    # 400, 403, 405, 500
    description: str

@dataclass
class EmptyPayload:
    """!
    @brief Payload nulo utilizado por mensajes de trazabilidad que no requieren datos.
    @details Usado exclusivamente por PING_REQ y ACK.
    """
    pass

# =========================================================================
# PAYLOADS DE RESPUESTA (Responses del servidor al cliente)
# =========================================================================

@dataclass
class BaseResponsePayload:
    """!
    @brief Estructura base de la cual heredan todas las respuestas del servidor.
    """
    success: bool
    code: int
    resp_type: str     # Ej: COMMAND_RESP, QUERY_RESP
    details: Optional[str] = None

@dataclass
class QueryRespPayload(BaseResponsePayload):
    """!
    @brief Respuesta que contiene los datos solicitados por una QUERY_REQ.
    @details Es altamente polimórfica: puede enviar listas de nombres, un diccionario
             con la telemetría del PC o la estructura compleja ROBOT_INFO.
    """
    data: Optional[Union[List[str], Dict[str, Any], List[Dict[str, Any]]]] = None

@dataclass
class ActionFeedbackPayload(BaseResponsePayload):
    """!
    @brief Respuesta de monitorización de progreso de Action Servers de ROS 2.
    """
    done_exec: Optional[bool] = None
    progress: Optional[int] = None
    status: Optional[str] = None

@dataclass
class StreamRespPayload(BaseResponsePayload):
    """!
    @brief Respuesta para abrir el canal de vídeo o enviar un paquete de sensor.
    """
    stream_data: Optional[Union[SensorEnvelope, str]] = None 
    stream_url: Optional[str] = None

@dataclass
class GenericRespPayload(BaseResponsePayload):
    """!
    @brief Respuesta básica de confirmación (ACK) para acciones que no requieren devolver datos.
    @details Utilizada por COMMAND_RESP, CONTROL_MODE_RESP, CONTROL_RESP y STOP_STREAM_RESP.
    """
    pass

# =========================================================================
# CONTENEDOR PRINCIPAL
# =========================================================================

@dataclass
class RobotMessage:
    """!
    @brief Clase maestra que representa cualquier trama de red del protocolo.
    @details Une la cabecera genérica con su payload de datos específico.
    """
    header: MessageHeader
    payload: Union[
        # Tipos de Request y Eventos
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
        # Tipos de Response
        QueryRespPayload,
        ActionFeedbackPayload,
        StreamRespPayload,
        GenericRespPayload
    ]