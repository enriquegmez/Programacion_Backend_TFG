"""
router.py
El Cerebro y Jefe de Sala del protocolo.
Orquesta la validación, comprueba los estados y ejecuta las acciones.
"""

import asyncio
import logging
import time
from typing import cast, Any
from app_tiago.utils.constants import MsgType, Action, StatusCode, RespType
from app_tiago.protocol.models import (
    RobotMessage, MessageHeader, GenericRespPayload, 
    ProtocolErrorPayload, EmptyPayload, AsyncNotifyPayload
)
from app_tiago.protocol.json_translator import MessageCodec
from app_tiago.protocol.models import CommandReqPayload, ControlModeReqPayload, ControlReqPayload

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
        self.last_control_req_arrival = 0.0

        # Watchdog interno para ROS 2
        self.is_monitoring = False
        self.monitor_task = None

    # ==========================================
    # EL WATCHDOG (Perro Guardián ROS 2)
    # ==========================================
    async def _monitor_robot_connection(self, send_callback, session_id):
        self.logger.info("Watchdog iniciado: Vigilando conexión con el robot...")
        
        while self.is_monitoring:
            await asyncio.sleep(2.0)
            
            if self.ros_node and not self.ros_node.check_connection():
                self.logger.error("¡Watchdog detecta pérdida de conexión con el robot!")
                self.is_monitoring = False
                
                self.ros_node.disconnect_from_robot()
                
                notify_header = MessageHeader(
                    msg_id=0, # Auto-generado por el codec
                    type=MsgType.ASYNC_NOTIFY,
                    session_id=session_id
                )
                
                # Usamos el payload correcto del Schema para notificaciones asíncronas
                notify_payload = AsyncNotifyPayload(
                    type="EMERGENCY_STOP", 
                    details="ROBOT_CONNECTION_LOST",
                    severity="CRITICAL"
                )
                
                msg = RobotMessage(header=notify_header, payload=notify_payload)
                
                self.state_machine.trigger_session_reset()
                await self._send_msg(msg, send_callback)
                break 

    async def send_session_assigned(self, session_id: str, send_callback):
        header = MessageHeader(msg_id=0, type=MsgType.ASYNC_NOTIFY, session_id=session_id)
        payload = AsyncNotifyPayload(type="session_id", details=f"SESSION_ASSIGNED:{session_id}")
        await self._send_msg(RobotMessage(header=header, payload=payload), send_callback)

    # ==========================================
    # ENTRADA PRINCIPAL DESDE EL SERVIDOR
    # ==========================================
    async def handle_raw_message(self, raw_string: str, send_callback, close_callback):
        # 1. ADUANA
        msg = self.codec.decode(raw_string)
        
        if msg.header.type == MsgType.PROTOCOL_ERROR:
            if msg.header.msg_id == -1:
                await self._send_msg(msg, send_callback)
            else:
                self.logger.error(f"El cliente reportó un error de protocolo: {msg.payload.description}")
            return

        # 2. SEGURIDAD DE SESIÓN
        if not self.connection_manager.is_valid_session(msg.header.session_id):
            self.logger.warning(f"Intento de acceso con session_id inválido: {msg.header.session_id}")
            error_msg = self._build_error_msg(StatusCode.FORBIDDEN, "Invalid session_id", msg.header.msg_id)
            await self._send_msg(error_msg, send_callback)
            return

        # 3. SEMÁFORO (Solo validación visual)
        is_valid, code, error_desc = self.state_machine.can_transition(msg)
        if not is_valid:
            error_msg = self._build_error_msg(code, error_desc, msg.header.msg_id, msg.header.session_id)
            await self._send_msg(error_msg, send_callback)
            return

        # 4. DISTRIBUCIÓN
        try:
            await self._route_message(msg, send_callback, close_callback)
        except Exception as e:
            self.logger.error(f"Error interno procesando el mensaje: {e}")
            error_msg = self._build_error_msg(StatusCode.INTERNAL_ERROR, "Internal server error", msg.header.msg_id, msg.header.session_id)
            await self._send_msg(error_msg, send_callback)

    # ==========================================
    # ENRUTADOR INTERNO
    # ==========================================
    async def _route_message(self, msg: RobotMessage, send_callback, close_callback):
        msg_type = msg.header.type
        session_id = msg.header.session_id
        req_msg_id = msg.header.msg_id

        # Cabecera genérica (Eco del ID)
        resp_header = MessageHeader(msg_id=req_msg_id, type=MsgType.RESP, session_id=session_id)
        resp_payload: Any = None

        match msg_type:
            
            case MsgType.PING_REQ:
                self.connection_manager.notify_ping_received()
                resp_header.type = MsgType.ACK
                resp_payload = EmptyPayload()

            case MsgType.COMMAND_REQ:
                cmd_payload = cast(CommandReqPayload, msg.payload) 
                action = cmd_payload.action
                match action:
                    case Action.CONNECT:
                        try:
                            # Intentamos conectar con el hardware/DDS
                            if self.ros_node and not self.ros_node.connect_to_robot():
                                # El nodo devuelve False (ej. timeout buscando tópicos)
                                resp_payload = GenericRespPayload(
                                    success=False, code=StatusCode.NOT_FOUND, 
                                    resp_type=RespType.COMMAND_RESP, details="Robot Tiago no detectado en la red."
                                )
                            else:
                                self.logger.info("Conexión exitosa con el robot.")
                                resp_payload = GenericRespPayload(
                                    success=True, code=StatusCode.OK, resp_type=RespType.COMMAND_RESP
                                )
                        except Exception as e:
                            resp_payload = GenericRespPayload(
                                success=False, code=StatusCode.INTERNAL_ERROR, 
                                resp_type=RespType.COMMAND_RESP, details=f"Error al conectar: {str(e)}"
                            )

                    case Action.DISCONNECT:
                        try:
                            if self.ros_node:
                                self.ros_node.stop_robot()
                                self.ros_node.disconnect_from_robot()
                                
                            # Si llega aquí, es que no ha habido errores
                            resp_payload = GenericRespPayload(
                                success=True, code=StatusCode.OK, resp_type=RespType.COMMAND_RESP
                            )
                        except Exception as e:
                            # Creamos el payload de error DENTRO del except, donde 'e' todavía existe
                            resp_payload = GenericRespPayload(
                                success=False, code=StatusCode.INTERNAL_ERROR, 
                                resp_type=RespType.COMMAND_RESP, details=f"Error al desconectar: {str(e)}"
                            )

                    case Action.END:
                        # Cortar conexión es seguro, no suele fallar a nivel lógico
                        resp_payload = GenericRespPayload(
                            success=True, code=StatusCode.OK, resp_type=RespType.COMMAND_RESP
                        )

            case MsgType.CONTROL_MODE_REQ:
                # SOLUCIÓN: Usamos cast para que Mypy sepa qué Payload es
                ctrl_mode_payload = cast(ControlModeReqPayload, msg.payload)
                event = ctrl_mode_payload.event
                # Usamos el topic relativo (sin barra) como vimos antes
                # Extraemos el valor que venga. Si es None o una cadena vacía "", usará 'cmd_vel'
                raw_topic = getattr(msg.payload, 'topic', None)
                topic = raw_topic if raw_topic else "cmd_vel"
                
                try:
                    if self.ros_node:
                        # Extraemos la tupla con el resultado y el mensaje de error
                        success, error_msg = self.ros_node.set_control_mode(event, topic)
                        self.last_control_req_arrival = 0.0  # Reset del watchdog de intervalo
                        
                        if not success:
                            # 1. Topic no válido o robot desconectado
                            resp_payload = GenericRespPayload(
                                success=False, code=StatusCode.BAD_REQUEST, 
                                resp_type=RespType.CONTROL_MODE_RESP, details=error_msg
                            )
                        else:
                            # 2. Todo OK, empezamos a publicar
                            resp_payload = GenericRespPayload(
                                success=True, code=StatusCode.OK, resp_type=RespType.CONTROL_MODE_RESP
                            )
                    else:
                        # Falla porque el gestor ROS 2 no está instanciado
                        resp_payload = GenericRespPayload(
                            success=False, code=StatusCode.INTERNAL_ERROR, 
                            resp_type=RespType.CONTROL_MODE_RESP, details="Backend ROS 2 no disponible."
                        )
                        
                except Exception as err:
                    self.logger.error(f"Excepción en CONTROL_MODE_REQ: {err}")
                    resp_payload = GenericRespPayload(
                        success=False, code=StatusCode.INTERNAL_ERROR, 
                        resp_type=RespType.CONTROL_MODE_RESP, details=f"Excepción en el hardware: {str(err)}"
                    )

            case MsgType.CONTROL_REQ:
                ctrl_payload = cast(ControlReqPayload, msg.payload)
                v = ctrl_payload.data.v
                w = ctrl_payload.data.w
                
                current_time = time.time()
                publish_success = True

                # 🛡️ PROTECCIÓN POR INTERVALO DE LLEGADA (Arrival Watchdog) 🛡️
                # Calculamos cuánto tiempo ha pasado desde el ÚLTIMO mensaje de control que recibimos
                if self.last_control_req_arrival > 0:
                    time_since_last_packet = current_time - self.last_control_req_arrival
                else:
                    # Es el primer paquete de la sesión, no hay intervalo que medir
                    time_since_last_packet = 0.0

                # 1. Si el "hueco" de silencio entre paquetes es > 0.5s, hay un problema de red
                if time_since_last_packet > 0.6:
                    self.logger.warning(f"⚠️ HUECO DE RED DETECTADO: {time_since_last_packet:.2f}s sin recibir órdenes. Frenado de seguridad.")
                    publish_success = False
                    if self.ros_node:
                        self.ros_node.stop_robot()
                
                # 2. Si el intervalo es correcto, intentamos publicar
                else:
                    if self.ros_node:
                        try:
                            publish_success = self.ros_node.publish_velocity(v, w)
                        except Exception as e:
                            self.logger.error(f"Fallo crítico enviando velocidad: {e}")
                            publish_success = False
                    else:
                        publish_success = False

                # ACTUALIZAMOS el marcador de tiempo para el próximo paquete
                self.last_control_req_arrival = current_time

                # 3. GESTIÓN DE RESPUESTAS AL MÓVIL
                # Avisamos inmediatamente si hubo error de LAG o si toca enviar el ACK rutinario (cada 0.5s)
                if not publish_success or (current_time - self.last_control_resp_time >= self.CONTROL_RESP_INTERVAL):
                    self.last_control_resp_time = current_time
                    
                    if not publish_success:
                        # Si falló por lag, mandamos detalles específicos para que la UI lo pinte
                        resp_payload = GenericRespPayload(
                            success=False, 
                            code=StatusCode.INTERNAL_ERROR, 
                            resp_type=RespType.CONTROL_RESP, 
                            details=f"Frenado de seguridad: Red inestable (Salto de {time_since_last_packet:.2f}s)."
                        )
                    else:
                        resp_payload = GenericRespPayload(
                            success=True, 
                            code=StatusCode.OK, 
                            resp_type=RespType.CONTROL_RESP
                        )

            case MsgType.ACK:
                self.logger.debug(f"Recibido ACK del cliente para el msg_id: {req_msg_id}")

            case _:
                self.logger.warning(f"Mensaje {msg_type} rutado pero no implementado.")

        # ==========================================
        # EL COMMIT DE ESTADO Y ENVÍO FINAL
        # ==========================================
        if resp_payload:
            resp_msg = RobotMessage(header=resp_header, payload=resp_payload)
            
            # 1. Sincronizamos la máquina de estados con lo que acaba de suceder
            self.state_machine.commit_transition(msg, resp_msg)
            
            # 2. Enviamos la respuesta por el WebSocket
            await self._send_msg(resp_msg, send_callback)
            
            # 3. Novedad: Si era END y ha ido bien, llamamos a la función del servidor
            # SOLUCIÓN: Usamos cast de nuevo y nos aseguramos de que resp_payload sea GenericRespPayload
            if msg_type == MsgType.COMMAND_REQ:
                cmd_payload = cast(CommandReqPayload, msg.payload)
                if cmd_payload.action == Action.END:
                    # Comprobamos el success de forma segura con getattr
                    is_success = getattr(resp_payload, 'success', False)
                    if is_success:
                        self.logger.info("Solicitando a server.py el cierre del túnel WebSocket...")
                        await close_callback()

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