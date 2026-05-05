"""
router.py
El Cerebro y Jefe de Sala del protocolo.
Orquesta la validación, comprueba los estados y ejecuta las acciones.
"""

import asyncio
import logging
import time
from app_tiago.utils.constants import MsgType, Action, StatusCode, RespType
from app_tiago.protocol.models import (
    RobotMessage, MessageHeader, CommandRespPayload, 
    GenericRespPayload, ProtocolErrorPayload, EmptyPayload,
    AsyncNotifyPayload
)
from app_tiago.protocol.json_translator import MessageCodec

class MessageRouter:
    def __init__(self, connection_manager, state_machine, ros_node=None):
        self.logger = logging.getLogger("MessageRouter")
        self.connection_manager = connection_manager
        self.state_machine = state_machine
        self.ros_node = ros_node 
        self.codec = MessageCodec()
        
        # Temporizador para no saturar la red con ControlResp
        self.last_control_resp_time = 0.0
        self.CONTROL_RESP_INTERVAL = 0.5  # Segundos entre ACKs de movimiento

        # --- NUEVAS VARIABLES PARA EL WATCHDOG ---
        self.is_monitoring = False
        self.monitor_task = None

    # ==========================================
    # EL WATCHDOG (Perro Guardián)
    # ==========================================
    async def _monitor_robot_connection(self, send_callback, session_id):
        """Tarea en segundo plano que vigila el enlace con ROS 2 cada 2 segundos."""
        self.logger.info("Watchdog iniciado: Vigilando conexión con el robot...")
        
        while self.is_monitoring:
            # Esperamos 2 segundos sin bloquear el servidor web
            await asyncio.sleep(2.0)
            
            # Comprobamos en silencio
            if self.ros_node and not self.ros_node.check_connection():
                self.logger.error("¡Watchdog detecta pérdida de conexión con el robot!")
                self.is_monitoring = False
                
                # 1. Por seguridad, apagamos el estado interno
                self.ros_node.disconnect_from_robot()
                
                # 2. Preparamos el aviso asíncrono para el frontend
                notify_header = MessageHeader(
                    msg_id=int(time.time()), # Un ID generado al vuelo
                    type=MsgType.ASYNC_NOTIFY,
                    session_id=session_id
                )
                
                notify_payload = GenericRespPayload(
                    success=False,
                    code=StatusCode.NOT_FOUND,
                    resp_type=RespType.ASYNC_NOTIFY,
                    details="ROBOT_CONNECTION_LOST"
                )
                
                # 3. Enviamos el mensaje
                msg = RobotMessage(header=notify_header, payload=notify_payload)
                await self._send_msg(msg, send_callback)
                break # Salimos del bucle infinito

    async def send_session_assigned(self, session_id: str, send_callback):
        """Envia el session_id al cliente nada más abrir el WebSocket."""
        # msg_id=0 para que el codec asigne uno nuevo automáticamente
        header = MessageHeader(msg_id=0, type=MsgType.ASYNC_NOTIFY, session_id=session_id)
        payload = AsyncNotifyPayload(type="INFO", details=f"SESSION_ASSIGNED:{session_id}")
        await self._send_msg(RobotMessage(header=header, payload=payload), send_callback)

    async def handle_raw_message(self, raw_string: str, send_callback):
        # 1. ADUANA: Decodificar y validar
        msg = self.codec.decode(raw_string)
        
        # --- FIX: Evitar bucle infinito de PROTOCOL_ERROR ---
        if msg.header.type == MsgType.PROTOCOL_ERROR:
            if msg.header.msg_id == -1:
                # Error generado internamente (JSON malformado, etc.). Se lo enviamos al cliente.
                await self._send_msg(msg, send_callback)
            else:
                # Error enviado por el cliente (ej. el móvil se queja de algo)
                self.logger.error(f"El cliente reportó un error de protocolo: {msg.payload.description}")
            return

        # 2. SEGURIDAD LÓGICA: Validar session_id
        if not self.connection_manager.is_valid_session(msg.header.session_id):
            self.logger.warning(f"Intento de acceso con session_id inválido: {msg.header.session_id}")
            # Le ponemos msg.header.msg_id para hacer eco de su ID erróneo
            error_msg = self._build_error_msg(StatusCode.FORBIDDEN, "Invalid session_id", msg.header.msg_id)
            await self._send_msg(error_msg, send_callback)
            return

        # 3. SEMÁFORO: Validar la Máquina de Estados
        is_valid, code, error_desc = self.state_machine.check_and_transition(msg)
        if not is_valid:
            error_msg = self._build_error_msg(code, error_desc, msg.header.msg_id, msg.header.session_id)
            await self._send_msg(error_msg, send_callback)
            return

        # 4. DISTRIBUCIÓN
        try:
            await self._route_message(msg, send_callback)
        except Exception as e:
            self.logger.error(f"Error interno procesando el mensaje: {e}")
            error_msg = self._build_error_msg(StatusCode.INTERNAL_ERROR, "Internal server error", msg.header.msg_id, msg.header.session_id)
            await self._send_msg(error_msg, send_callback)

   # ==========================================
    # ENRUTADOR INTERNO (Con manejo de errores ROS 2)
    # ==========================================
    async def _route_message(self, msg: RobotMessage, send_callback):
        msg_type = msg.header.type
        session_id = msg.header.session_id
        req_msg_id = msg.header.msg_id

        # Cabecera genérica (Eco del ID)
        resp_header = MessageHeader(msg_id=req_msg_id, type=MsgType.RESP, session_id=session_id)
        resp_payload = None

        match msg_type:
            
            case MsgType.PING_REQ:
                self.connection_manager.notify_ping_received()
                resp_header.type = MsgType.ACK
                resp_payload = EmptyPayload()

            case MsgType.COMMAND_REQ:
                match msg.payload.action:
                    case Action.CONNECT:
                        try:
                            # Intentamos conectar con el hardware/DDS
                            if self.ros_node and not self.ros_node.connect_to_robot():
                                # El nodo devuelve False (ej. timeout buscando tópicos)
                                resp_payload = CommandRespPayload(
                                    success=False, code=StatusCode.NOT_FOUND, 
                                    resp_type=RespType.COMMAND_RESP, details="Robot Tiago no detectado en la red."
                                )
                            else:
                                self.logger.info("Conexión exitosa con el robot.")
                                resp_payload = CommandRespPayload(
                                    success=True, code=StatusCode.OK, resp_type=RespType.COMMAND_RESP
                                )
                        except Exception as e:
                            resp_payload = CommandRespPayload(
                                success=False, code=StatusCode.INTERNAL_ERROR, 
                                resp_type=RespType.COMMAND_RESP, details=f"Error al conectar: {str(e)}"
                            )

                    case Action.DISCONNECT:
                        try:
                            if self.ros_node:
                                self.ros_node.disconnect_from_robot()
                            resp_payload = CommandRespPayload(
                                success=True, code=StatusCode.OK, resp_type=RespType.COMMAND_RESP
                            )
                        except Exception as e:
                            resp_payload = CommandRespPayload(
                                success=False, code=StatusCode.INTERNAL_ERROR, 
                                resp_type=RespType.COMMAND_RESP, details=f"Error al desconectar: {str(e)}"
                            )

                    case Action.END:
                        # Cortar conexión es seguro, no suele fallar a nivel lógico
                        resp_payload = CommandRespPayload(
                            success=True, code=StatusCode.OK, resp_type=RespType.COMMAND_RESP
                        )

            case MsgType.CONTROL_MODE_REQ:
                event = msg.payload.event
                # Usamos el topic relativo (sin barra) como vimos antes
                topic = getattr(msg.payload, 'topic', 'cmd_vel')
                
                try:
                    if self.ros_node:
                        # Extraemos la tupla con el resultado y el mensaje de error
                        success, error_msg = self.ros_node.set_control_mode(event, topic)
                        
                        if not success:
                            # 1. Topic no válido o robot desconectado
                            resp_payload = GenericRespPayload(
                                success=False, 
                                code=StatusCode.BAD_REQUEST, 
                                resp_type=RespType.CONTROL_MODE_RESP, 
                                details=error_msg  # <-- Enviamos el motivo exacto al frontend
                            )
                        else:
                            # 2. Todo OK, empezamos a publicar
                            resp_payload = GenericRespPayload(
                                success=True, 
                                code=StatusCode.OK, 
                                resp_type=RespType.CONTROL_MODE_RESP
                            )
                    else:
                        # Falla porque el gestor ROS 2 no está instanciado
                        resp_payload = GenericRespPayload(
                            success=False, 
                            code=StatusCode.INTERNAL_ERROR, 
                            resp_type=RespType.CONTROL_MODE_RESP, 
                            details="Backend ROS 2 no disponible."
                        )
                        
                except Exception as e:
                    self.logger.error(f"Excepción en CONTROL_MODE_REQ: {e}")
                    resp_payload = GenericRespPayload(
                        success=False, 
                        code=StatusCode.INTERNAL_ERROR, 
                        resp_type=RespType.CONTROL_MODE_RESP, 
                        details=f"Excepción en el hardware: {str(e)}"
                    )

            case MsgType.CONTROL_REQ:
                v = msg.payload.data.v
                w = msg.payload.data.w
                
                publish_success = True
                if self.ros_node:
                    try:
                        publish_success = self.ros_node.publish_velocity(v, w)
                    except Exception as e:
                        self.logger.error(f"Fallo crítico enviando velocidad: {e}")
                        publish_success = False

                # THROTTLING: Solo enviamos respuesta si ha pasado X tiempo
                current_time = time.time()
                if current_time - self.last_control_resp_time >= self.CONTROL_RESP_INTERVAL:
                    self.last_control_resp_time = current_time
                    
                    if publish_success is False:
                        # Avisamos al frontend de que el robot no está recibiendo los datos
                        resp_payload = GenericRespPayload(
                            success=False, code=StatusCode.INTERNAL_ERROR, 
                            resp_type=RespType.CONTROL_RESP, details="Fallo publicando en /cmd_vel."
                        )
                    else:
                        resp_payload = GenericRespPayload(
                            success=True, code=StatusCode.OK, resp_type=RespType.CONTROL_RESP
                        )

            case MsgType.ACK:
                self.logger.debug(f"Recibido ACK del cliente para el msg_id: {req_msg_id}")

            case _:
                self.logger.warning(f"Mensaje {msg_type} rutado pero no implementado.")

        # --- ENVÍO FINAL ---
        if resp_payload:
            resp_msg = RobotMessage(header=resp_header, payload=resp_payload)
            await self._send_msg(resp_msg, send_callback)

    # ==========================================
    # UTILIDADES
    # ==========================================
    async def _send_msg(self, msg: RobotMessage, send_callback):
        json_str = self.codec.encode(msg)
        await send_callback(json_str)

    def _build_error_msg(self, code: int, description: str, req_msg_id: int, session_id: str = "") -> RobotMessage:
        header = MessageHeader(msg_id=req_msg_id, type=MsgType.PROTOCOL_ERROR, session_id=session_id)
        payload = ProtocolErrorPayload(error_code=code, description=description)
        return RobotMessage(header=header, payload=payload)