## @file director.py
#  @brief El Cerebro y Director de Orquesta del protocolo.
#  @details Orquesta la validación de mensajes, evalúa las transiciones de la máquina de estados 
#           y deriva las acciones físicas tanto a ROS 2 como al sistema operativo del host.
#  @author Enrique Gómez
#  @date 2026

import asyncio
import logging
import time
from typing import cast, Any, Optional

from r2pilot_server.utils.constants import MsgType, Action, StatusCode, RespType, Resource, StreamConfig, SessionTimeout
from r2pilot_server.protocol.models import (
    RobotMessage, MessageHeader, GenericRespPayload, 
    ProtocolErrorPayload, EmptyPayload, AsyncNotifyPayload,
    ActionFeedbackPayload, CommandReqPayload, ActionReqPayload, 
    ControlModeReqPayload, ControlReqPayload, StreamReqPayload, 
    StopStreamReqPayload, StreamRespPayload, QueryReqPayload, 
    QueryRespPayload, StopActionReqPayload
)
from r2pilot_server.protocol.message_codec import MessageCodec
from r2pilot_server.core.host_utils import HostSystemManager


class Director:
    """!
    @brief Gestor central de enrutamiento lógico de paquetes.
    @details Recibe JSONs decodificados, comprueba permisos, invoca a los nodos 
             inferiores y construye las respuestas asíncronas para el cliente.
    """

    def __init__(self, connection_manager: Any, state_machine: Any, ros_node: Optional[Any] = None) -> None:
        """!
        @brief Constructor del enrutador principal del protocolo.
        @param connection_manager Gestor de la sesión lógica.
        @param state_machine Máquina de estados finitos del protocolo de comunicaciones.
        @param ros_node Puente de comunicación bidireccional con el ecosistema de ROS 2.
        """
        self.logger = logging.getLogger("Director")
        self.connection_manager = connection_manager
        self.state_machine = state_machine
        self.ros_node = ros_node 
        
        # Instanciamos el traductor de JSON y nuestro nuevo gestor físico aislado de Linux
        self.codec = MessageCodec()
        self.system_manager = HostSystemManager()
        
        # Configuración del control de latencia del robot
        self.CONTROL_RESP_INTERVAL = SessionTimeout.CONTROL_RESP_INTERVAL
        self.last_control_resp_time = 0.0
        self.last_control_req_arrival = 0.0
        self.watchdog_triggered = False

        # Control del Watchdog de ROS 2
        self.is_monitoring = False
        self.monitor_task: Optional[asyncio.Task[Any]] = None

        # ---------------------------------------------------------------------
        # [TFG] CAMBIO 1: Referencia a la tarea asíncrona del Watchdog de Control
        # ---------------------------------------------------------------------
        self.control_watchdog_task: Optional[asyncio.Task[Any]] = None

    # =========================================================================
    # VIGILANCIA ASÍNCRONA DE ROS 2 (Watchdog)
    # =========================================================================

    async def _monitor_robot_connection(self, send_callback: Any, session_id: str) -> None:
        """!
        @brief Bucle en segundo plano que monitoriza la presencia e integridad del robot en ROS 2.
        @details Si el nodo ROS2 pierde comunicación con el robot o detecta choques de IPs
                 de otros robots en la red, detiene la sesión de forma proactiva para evitar accidentes.
        @param send_callback Clausura de envío para avisar al terminal móvil de inmediato.
        @param session_id Token de la sesión afectada que se va a eliminar.
        @return None
        """
        self.logger.info("[WATCHDOG ROS] Vigilando presencia física y exclusión de robots en red...")
        
        while self.is_monitoring:
            await asyncio.sleep(2.0)
            
            if self.ros_node:
                status = self.ros_node.check_connection()
                
                # 0 = Conexión perdida | 2 = Conflicto de múltiples robots detectados
                if status in [0, 2]:
                    self.is_monitoring = False
                    self.ros_node.disconnect_from_robot()
                    
                    if status == 2:
                        self.logger.error("[WATCHDOG ROS] MÚLTIPLES ROBOTS activos detectados. Abortando control.")
                        error_detail = "MULTIPLE_ROBOTS_DETECTED"
                    else:
                        self.logger.error("[WATCHDOG ROS] Pérdida total de comunicación física con el robot.")
                        error_detail = "ROBOT_CONNECTION_LOST"
                        
                    notify_msg = RobotMessage(
                        header=MessageHeader(msg_id=0, type=MsgType.ASYNC_NOTIFY, session_id=session_id),
                        payload=AsyncNotifyPayload(type="EMERGENCY_STOP", details=error_detail, severity="CRITICAL")
                    )
                    
                    self.state_machine.trigger_session_reset()
                    await self._send_msg(notify_msg, send_callback)
                    break


    async def _control_watchdog_loop(self) -> None:
        """!
        @brief Watchdog de Seguridad de Control (Deadman Switch) [TFG].
        @details Nivel 2 de seguridad: Detecta cortes totales de red proactivamente.
                 Frena el hardware de forma silenciosa a los 1.0s. La sincronización
                 lógica de estados se delega al filtro reactivo de 0.6s.
        """
        self.logger.info("[SEGURIDAD] Watchdog de Control silencioso iniciado (Límite: 1.0s)")
        try:
            while self.is_monitoring:
                await asyncio.sleep(0.1) # Evaluación rápida a 10 Hz
                
                if self.last_control_req_arrival > 0:
                    time_since_last = time.time() - self.last_control_req_arrival
                    
                    if time_since_last > 1.0:
                        # Si es la primera vez que detecta este corte, frena y mide
                        if not self.watchdog_triggered:
                            t1_deadman = time.perf_counter_ns()
                            
                            self.logger.error(
                                f"[SEGURIDAD CRÍTICA] Silencio de {time_since_last:.2f}s sin comandos. "
                                f"Frenando hardware silenciosamente."
                            )
                            
                            if self.ros_node:
                                self.ros_node.stop_robot()
                                
                            t2_deadman = time.perf_counter_ns()
                            latencia_ms = (t2_deadman - t1_deadman) / 1_000_000.0
                            
                            self.logger.warning(
                                f"\n==================================================\n"
                                f"[MÉTRICA TFG - DEADMAN] Tiempo de reacción del freno: {latencia_ms:.4f} ms\n"
                                f"=================================================="
                            )
                            
                            # Levantamos el flag para no repetir esto 10 veces por segundo
                            self.watchdog_triggered = True
                            
                        # CRÍTICO: NO reseteamos last_control_req_arrival. 
                        # Dejamos que el filtro reactivo vea el tiempo real de desconexión.
                        
        except asyncio.CancelledError:
            self.logger.debug("[SEGURIDAD] Watchdog de Control detenido limpiamente.")

    # =========================================================================
    # PROCESAMIENTO DE MENSAJES 
    # =========================================================================

    async def send_session_assigned(self, session_id: str, send_callback: Any) -> None:
        """!
        @brief Emite la notificación asíncrona inicial de autorización de sesión.
        @param session_id Token UUIDv4 asignado para esta conexión.
        @param send_callback Clausura física de envío de la red.
        @return None
        """
        header = MessageHeader(msg_id=0, type=MsgType.ASYNC_NOTIFY, session_id=session_id)
        payload = AsyncNotifyPayload(type="session_id", details=f"SESSION_ASSIGNED:{session_id}")
        await self._send_msg(RobotMessage(header=header, payload=payload), send_callback)

    async def handle_raw_message(self, raw_string: str, send_callback: Any, close_callback: Any) -> None:
        """!
        @brief Punto de entrada de paquetes del servidor WebSocket.
        @details Filtra los errores de formato, comprueba la validez del token de sesión
                 y valida si el comando respeta las transiciones de la máquina de estados.
        @param raw_string Cadena de texto JSON cruda recibida de la red.
        @param send_callback Clausura para emitir respuestas de vuelta.
        @param close_callback Clausura para solicitar cierres físicos del socket.
        @return None
        """
        # 1. Traducción del paquete (JSON -> Objetos de Python)
        msg = self.codec.decode(raw_string)
        
        if msg.header.type == MsgType.PROTOCOL_ERROR:
            error_payload = cast(ProtocolErrorPayload, msg.payload)
            if msg.header.msg_id == -1:
                await self._send_msg(msg, send_callback)
            else:
                self.logger.error(f"[CLIENTE] Error de protocolo reportado por el terminal: {error_payload.description}")
            return

        # 2. Validación de sesión
        if not self.connection_manager.is_valid_session(msg.header.session_id):
            self.logger.warning(f"[SEGURIDAD] Intento de intrusión (session_id inválido): {msg.header.session_id}")
            error_msg = self._build_error_msg(StatusCode.FORBIDDEN, "Invalid session_id", msg.header.msg_id)
            await self._send_msg(error_msg, send_callback)
            return

        # 3. Validación de transiciones lógicas de la máquina de estados
        is_valid, code, error_desc = self.state_machine.can_transition(msg)
        if not is_valid:
            error_msg = self._build_error_msg(code, error_desc, msg.header.msg_id, msg.header.session_id)
            await self._send_msg(error_msg, send_callback)
            return

        # 4. Derivación del procesamiento
        try:
            await self._route_message(msg, send_callback, close_callback)
        except Exception as e:
            self.logger.error(f"[ROUTER] Excepción interna al enrutar mensaje: {e}")
            error_msg = self._build_error_msg(StatusCode.INTERNAL_ERROR, "Internal server error", msg.header.msg_id, msg.header.session_id)
            await self._send_msg(error_msg, send_callback)

    async def _route_message(self, msg: RobotMessage, send_callback: Any, close_callback: Any) -> None:
        """!
        @brief Enrutador semántico de comandos.
        @details Analiza el tipo de mensaje (PING, COMMAND, QUERY, etc.) y ejecuta la acción.
        @param msg Instancia del mensaje de protocolo validado.
        @param send_callback Clausura de envío del canal.
        @param close_callback Clausura de cierre limpio de red.
        @return None
        """
        msg_type = msg.header.type
        session_id = msg.header.session_id
        req_msg_id = msg.header.msg_id

        # Cabecera genérica de respuesta
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
                            if not self.ros_node:
                                resp_payload = GenericRespPayload(
                                    success=False, code=StatusCode.INTERNAL_ERROR, 
                                    resp_type=RespType.COMMAND_RESP, details="Backend ROS 2 no inicializado."
                                )
                            else:
                                connection_status = self.ros_node.connect_to_robot()

                                # 0 = Conexión perdida | 1 = Conexión establecida | 2 = Conflicto de múltiples robots detectados
                                if connection_status == 0:
                                    resp_payload = GenericRespPayload(
                                        success=False, code=StatusCode.NOT_FOUND, 
                                        resp_type=RespType.COMMAND_RESP, details="Robot no detectado en la red."
                                    )
                                elif connection_status == 2:
                                    resp_payload = GenericRespPayload(
                                        success=False, code=StatusCode.BAD_REQUEST, 
                                        resp_type=RespType.COMMAND_RESP, details="Conflicto: Múltiples robots detectados."
                                    )
                                elif connection_status == 1:
                                    self.logger.info("[ROS] Conexión lógica establecida con el robot.")
                                    if not self.is_monitoring:
                                        self.is_monitoring = True
                                        self.monitor_task = asyncio.create_task(
                                            self._monitor_robot_connection(send_callback, session_id)
                                        )

                                        # -----------------------------------------------------------------
                                        # [TFG] CAMBIO 3: Arrancar el Watchdog de control al conectarse
                                        # -----------------------------------------------------------------
                                        self.control_watchdog_task = asyncio.create_task(
                                            self._control_watchdog_loop()
                                        )

                                    resp_payload = GenericRespPayload(
                                        success=True, code=StatusCode.OK, resp_type=RespType.COMMAND_RESP
                                    )
                        except Exception as e:
                            resp_payload = GenericRespPayload(
                                success=False, code=StatusCode.INTERNAL_ERROR, 
                                resp_type=RespType.COMMAND_RESP, details=f"Error al conectar: {str(e)}"
                            )

                    case Action.CHANGE_VARS:
                        domain_id = cmd_payload.param1 or "0"
                        dds = cmd_payload.param2 or "rmw_cyclonedds_cpp"
                        use_discovery = cmd_payload.param3 if cmd_payload.param3 is not None else False
                        
                        # DELEGACIÓN AL GESTOR DE SISTEMA
                        if self.system_manager.write_env_file(domain_id, dds, use_discovery):
                            resp_payload = GenericRespPayload(success=True, code=StatusCode.OK, resp_type=RespType.COMMAND_RESP)
                        else:
                            resp_payload = GenericRespPayload(
                                success=False, code=StatusCode.INTERNAL_ERROR, 
                                resp_type=RespType.COMMAND_RESP, details="Fallo escribiendo variables de entorno en host."
                            )

                    case Action.REBOOT | Action.SHUTDOWN:
                        self.logger.info(f"[SISTEMA] Iniciando secuencia de parada controlada para: {action}...")
                        #self.is_monitoring = False
                        #if self.monitor_task: self.monitor_task.cancel()

                        # -----------------------------------------------------------------
                        # [TFG] CAMBIO 4: Cancelar la tarea de seguridad al apagar/reiniciar
                        # -----------------------------------------------------------------
                        self.is_monitoring = False
                        if self.monitor_task: self.monitor_task.cancel()
                        if self.control_watchdog_task: self.control_watchdog_task.cancel()

                        if self.ros_node:
                            try:
                                self.ros_node.stop_robot()
                                self.ros_node.disconnect_from_robot()
                            except Exception as e:
                                self.logger.warning(f"[ROS] Aviso al frenar ROS antes de apagar: {e}")
                        
                        resp_payload = GenericRespPayload(
                            success=True, code=StatusCode.OK, resp_type=RespType.COMMAND_RESP
                        )
                        # DELEGACIÓN AL GESTOR DE SISTEMA (Mandar orden al kernel de Linux)
                        asyncio.create_task(asyncio.to_thread(self.system_manager.execute_power_command, action))  

                    case Action.DISCONNECT:
                        try:
                            # -----------------------------------------------------------------
                            # [TFG] CAMBIO 4: Cancelar la tarea de seguridad al desconectar
                            # -----------------------------------------------------------------
                            self.is_monitoring = False
                            if self.monitor_task: self.monitor_task.cancel()
                            if self.control_watchdog_task: self.control_watchdog_task.cancel()

                            #self.is_monitoring = False
                            #if self.monitor_task: self.monitor_task.cancel()

                            if self.ros_node:
                                self.ros_node.stop_robot()
                                self.ros_node.disconnect_from_robot()
                                
                            resp_payload = GenericRespPayload(
                                success=True, code=StatusCode.OK, resp_type=RespType.COMMAND_RESP
                            )
                        except Exception as e:
                            resp_payload = GenericRespPayload(
                                success=False, code=StatusCode.INTERNAL_ERROR, 
                                resp_type=RespType.COMMAND_RESP, details=f"Error al desconectar: {str(e)}"
                            )

                    case Action.END:
                        self.is_monitoring = False
                        if self.monitor_task: self.monitor_task.cancel()
                            
                        resp_payload = GenericRespPayload(
                            success=True, code=StatusCode.OK, resp_type=RespType.COMMAND_RESP
                        )

            case MsgType.QUERY_REQ:
                query_payload = cast(QueryReqPayload, msg.payload)
                
                # DELEGACIÓN AL GESTOR DE SISTEMA (Telemetría de la sala de espera)
                if query_payload.resource_type == Resource.HOST_INFO:
                    telemetry_data = self.system_manager.get_host_telemetry()
                    resp_payload = QueryRespPayload(
                        success=True, code=StatusCode.OK, 
                        resp_type=RespType.QUERY_RESP, data=telemetry_data
                    )
                
                elif not self.ros_node:
                    resp_payload = QueryRespPayload(
                        success=False, code=StatusCode.INTERNAL_ERROR, 
                        resp_type=RespType.QUERY_RESP, details="ROS 2 no disponible."
                    )
                
                #Acciones del módulo PlayMotion
                elif query_payload.resource_type == Resource.MOVEMENTS:
                    if self.ros_node:
                        success, action_list_or_error = self.ros_node.get_available_actions()
                        resp_payload = QueryRespPayload(
                            success=success,
                            code=StatusCode.OK if success else StatusCode.INTERNAL_ERROR,
                            resp_type=RespType.QUERY_RESP,
                            data=action_list_or_error if success else None,
                            details=str(action_list_or_error) if not success else None
                        )
                
                #Topics de ROS2
                elif query_payload.resource_type == Resource.TOPICS:
                    if self.ros_node:
                        data = self.ros_node.get_all_topics()
                        resp_payload = QueryRespPayload(
                            success=bool(data),
                            code=StatusCode.OK if data else StatusCode.NOT_FOUND, 
                            resp_type=RespType.QUERY_RESP,
                            data=data if data else None,
                            details="No se encontraron topics." if not data else None
                        )
                
                #Servicios de ROS2
                elif query_payload.resource_type == Resource.SERVICES:
                    if self.ros_node:
                        data = self.ros_node.get_all_services()
                        resp_payload = QueryRespPayload(
                            success=bool(data),
                            code=StatusCode.OK if data else StatusCode.NOT_FOUND, 
                            resp_type=RespType.QUERY_RESP,
                            data=data if data else None,
                            details="No se encontraron servicios." if not data else None
                        )

                #Acciones de ROS2
                elif query_payload.resource_type == Resource.ACTIONS:
                    if self.ros_node:
                        data = self.ros_node.get_all_actions()
                        resp_payload = QueryRespPayload(
                            success=bool(data),
                            code=StatusCode.OK if data else StatusCode.NOT_FOUND, 
                            resp_type=RespType.QUERY_RESP,
                            data=data if data else None,
                            details="No se encontraron acciones." if not data else None
                        )
                
                #Sensores del robot disponibles
                elif query_payload.resource_type == Resource.SENSORS:
                    if self.ros_node:
                        data = self.ros_node.get_available_sensors()
                        resp_payload = QueryRespPayload(
                            success=bool(data),
                            code=StatusCode.OK if data else StatusCode.NOT_FOUND, 
                            resp_type=RespType.QUERY_RESP,
                            data=data if data else None,
                            details="No se detectaron sensores compatibles." if not data else None
                        )
                
                #Información de capacidades del robot (hardware y software)
                elif query_payload.resource_type == Resource.ROBOT_INFO:
                    resp_data = self.ros_node.get_robot_capabilities()
                    resp_payload = QueryRespPayload(
                        success=True, code=StatusCode.OK, 
                        resp_type=RespType.QUERY_RESP, data=resp_data
                    )
                else:
                    resp_payload = QueryRespPayload(
                        success=False, code=StatusCode.NOT_ALLOWED, 
                        resp_type=RespType.QUERY_RESP, details=f"Recurso '{query_payload.resource_type}' desconocido."
                    )
            
            case MsgType.ACTION_REQ:
                action_payload = cast(ActionReqPayload, msg.payload)
                if self.ros_node:
                    main_loop = asyncio.get_running_loop()

                    async def _sync_and_send_feedback(fb_msg: RobotMessage) -> None:
                        """!
                        @brief Sincroniza el estado lógico y envía el feedback asíncrono.
                        @details Sirve de puente para que las respuestas del Action Server de ROS 2 
                                 pasen de forma segura al bucle de eventos de la red física.
                        @param fb_msg El mensaje de protocolo empaquetado con el estado de la acción.
                        @return None
                        """
                        self.state_machine.commit_transition(msg, fb_msg)
                        await self._send_msg(fb_msg, send_callback)

                    def ros2_feedback_handler(is_success: bool, done_exec: bool, progress: int, status: str) -> None:
                        """!
                        @brief Clausura inyectable para monitorizar el Action Server de ROS 2.
                        @details Traduce las variables crudas de estado de ROS 2 al estándar del protocolo 
                                 R2Pilot (ActionFeedbackPayload) y programa su envío sin bloquear el hilo.
                        @param is_success Indica si el Action Server aceptó o está ejecutando bien la tarea.
                        @param done_exec Indica si la tarea ha terminado (ya sea por éxito, error o cancelación).
                        @param progress Porcentaje de completado (0-100).
                        @param status Texto descriptivo del estado actual del robot.
                        @return None
                        """
                        fb_payload = ActionFeedbackPayload(
                            success=is_success,
                            code=StatusCode.OK if is_success else StatusCode.INTERNAL_ERROR,
                            resp_type=RespType.ACTION_FEEDBACK,
                            details=status,
                            status="completed" if (done_exec and is_success) else ("failed" if not is_success else "running"),
                            progress=progress,
                            done_exec=done_exec
                        )
                        fb_msg = RobotMessage(header=resp_header, payload=fb_payload)
                        asyncio.run_coroutine_threadsafe(_sync_and_send_feedback(fb_msg), main_loop)

                    #Ejecutamos la acción en ROS 2 y registramos el callback de feedback
                    self.ros_node.set_action_feedback_callback(ros2_feedback_handler)
                    success, msg_str = self.ros_node.execute_action(action_payload.type, action_payload.target)
                    
                    resp_payload = ActionFeedbackPayload(
                        success=success,
                        code=StatusCode.OK if success else StatusCode.NOT_ALLOWED,
                        resp_type=RespType.ACTION_FEEDBACK,
                        details=msg_str,
                        status="accepted" if success else "rejected",
                        progress=0,
                        done_exec=not success
                    )
                else:
                    resp_payload = ActionFeedbackPayload(
                        success=False, code=StatusCode.INTERNAL_ERROR, resp_type=RespType.ACTION_FEEDBACK,
                        details="Backend ROS 2 no disponible.", status="error", done_exec=True
                    )

            case MsgType.STOP_ACTION_REQ:
                stop_action_payload = cast(StopActionReqPayload, msg.payload)
                if self.ros_node:
                    stop_success = self.ros_node.stop_action(stop_action_payload.type, stop_action_payload.target)
                    resp_payload = GenericRespPayload(
                        success=stop_success, 
                        code=StatusCode.OK if stop_success else StatusCode.INTERNAL_ERROR, 
                        resp_type=RespType.STOP_ACTION_FEEDBACK, 
                        details="Acción detenida." if stop_success else "No se pudo detener la acción.",
                    ) 
                else:
                    resp_payload = GenericRespPayload(
                        success=False, code=StatusCode.INTERNAL_ERROR, resp_type=RespType.STOP_ACTION_FEEDBACK, 
                        details="Backend ROS 2 no disponible.",
                    )

            case MsgType.CONTROL_MODE_REQ:
                ctrl_mode_payload = cast(ControlModeReqPayload, msg.payload)
                event, control_type = ctrl_mode_payload.event, ctrl_mode_payload.type 
                raw_topic = getattr(msg.payload, 'topic', None)
                topic = raw_topic if raw_topic else "cmd_vel"
                
                try:
                    if self.ros_node:
                        success, error_msg = self.ros_node.set_control_mode(event, control_type, topic)
                        self.last_control_req_arrival = 0.0  
                        
                        resp_payload = GenericRespPayload(
                            success=success, 
                            code=StatusCode.OK if success else StatusCode.BAD_REQUEST, 
                            resp_type=RespType.CONTROL_MODE_RESP, 
                            details=error_msg if not success else None
                        )
                    else:
                        resp_payload = GenericRespPayload(
                            success=False, code=StatusCode.INTERNAL_ERROR, 
                            resp_type=RespType.CONTROL_MODE_RESP, details="Backend ROS 2 no disponible."
                        )
                except Exception as err:
                    self.logger.error(f"[HARDWARE] Excepción en CONTROL_MODE_REQ: {err}")
                    resp_payload = GenericRespPayload(
                        success=False, code=StatusCode.INTERNAL_ERROR, 
                        resp_type=RespType.CONTROL_MODE_RESP, details=f"Excepción en hardware: {str(err)}"
                    )

            case MsgType.CONTROL_REQ:
                ctrl_payload = cast(ControlReqPayload, msg.payload)
                v, w = ctrl_payload.data.v, ctrl_payload.data.w
                joint_name, joint_value = ctrl_payload.data.joint_name, ctrl_payload.data.joint_value
                
                current_time = time.time()
                publish_success, publish_error_msg = True, "" 
                
                # Determinamos si es control de teleoperación o de articulación
                is_teleop = not joint_name 

                if is_teleop:
                    # Control de latencia: Detectamos huecos de tiempo entre paquetes de control y avisamos al cliente
                    time_since_last_packet = (current_time - self.last_control_req_arrival) if self.last_control_req_arrival > 0 else 0.0

                    if time_since_last_packet > 0.6:
                        self.logger.warning(f"[RED] HUECO DE VELOCIDAD: {time_since_last_packet:.2f}s sin recibir órdenes.")
                        publish_success, publish_error_msg = False, f"Red inestable (Salto de {time_since_last_packet:.2f}s)."
                        if self.ros_node: self.ros_node.stop_robot()
                    else:
                        if self.ros_node:
                            try:
                                # Publicamos la velocidad lineal y angular en ROS 2
                                publish_success, publish_error_msg = self.ros_node.publish_velocity(v, w)
                            except Exception:
                                publish_success, publish_error_msg = False, "Error interno de velocidad."
                        else:
                            publish_success, publish_error_msg = False, "Backend ROS no disponible."
                    self.last_control_req_arrival = current_time
                else:
                    if self.ros_node:
                        try:
                            # Publicamos la posición de la articulación en ROS 2
                            publish_success = self.ros_node.publish_joint_position(joint_name, joint_value)
                            if not publish_success: publish_error_msg = "Error en articulación."
                        except Exception:
                            publish_success, publish_error_msg = False, "Excepción en articulación."
                    else:
                        publish_success, publish_error_msg = False, "Backend ROS no disponible."
                    self.last_control_req_arrival = current_time
                    self.watchdog_triggered = False  # Bajamos el flag
                
                # Control de frecuencia de respuesta: Solo enviamos un CONTROL_RESP cada CONTROL_RESP_INTERVAL segundos
                if not publish_success or (current_time - self.last_control_resp_time >= self.CONTROL_RESP_INTERVAL):
                    self.last_control_resp_time = current_time
                    resp_payload = GenericRespPayload(
                        success=publish_success, 
                        code=StatusCode.OK if publish_success else StatusCode.INTERNAL_ERROR, 
                        resp_type=RespType.CONTROL_RESP, 
                        details=publish_error_msg if not publish_success else None
                    )

            case MsgType.STREAM_REQ:
                stream_payload = cast(StreamReqPayload, msg.payload)
                
                # Usuario pide visualizar la cámara
                if stream_payload.resource == Resource.CAMERA:
                    topic = stream_payload.topic
                    if not topic:
                        resp_payload = StreamRespPayload(
                            success=False, code=StatusCode.BAD_REQUEST, resp_type=RespType.STREAM_RESP,
                            details="Topic obligatorio."
                        )
                    else:
                        if self.ros_node and not self.ros_node.is_video_server_running():
                            self.logger.warning("[VÍDEO] Rechazado: web_video_server apagado.")
                            resp_payload = StreamRespPayload(
                                success=False, code=StatusCode.INTERNAL_ERROR, resp_type=RespType.STREAM_RESP,
                                details="Servidor de vídeo no responde."
                            )
                        elif self.ros_node and not self.ros_node.is_topic_active(topic):
                            self.logger.warning(f"[VÍDEO] Rechazado: Topic {topic} no existe.")
                            resp_payload = StreamRespPayload(
                                success=False, code=StatusCode.NOT_FOUND, resp_type=RespType.STREAM_RESP,
                                details="Cámara desconectada o topic erróneo."
                            )
                        else:
                            # DELEGACIÓN AL GESTOR DE SISTEMA (Obtener IP de vídeo dinámica)
                            server_ip = self.system_manager.get_local_ip()
                            quality = stream_payload.quality_level if stream_payload.quality_level else "medium"
                            url_params = StreamConfig.CAMERA_PROFILES.get(quality, StreamConfig.CAMERA_PROFILES["medium"])
                            final_url = f"http://{server_ip}:8081/stream?topic={topic}{url_params}"
                            
                            self.logger.info(f"[VÍDEO] Stream autorizado. URL: {final_url}")
                            resp_payload = StreamRespPayload(
                                success=True, code=StatusCode.OK, resp_type=RespType.STREAM_RESP, stream_url=final_url
                            )

                # Usuario pide visualizar los sensores
                elif stream_payload.resource.upper() == Resource.SENSORS:
                    topic = stream_payload.topic
                    if not topic:
                        resp_payload = StreamRespPayload(
                            success=False, code=StatusCode.BAD_REQUEST, resp_type=RespType.STREAM_RESP,
                            details="Topic obligatorio."
                        )
                    elif self.ros_node:
                        if not self.ros_node.is_topic_active(topic):
                            resp_payload = StreamRespPayload(
                                success=False, code=StatusCode.NOT_FOUND, resp_type=RespType.STREAM_RESP,
                                details=f"Sensor {topic} no existe."
                            )
                        else:
                            main_loop = asyncio.get_running_loop()

                            async def _send_sensor_data(sensor_obj: Any) -> None:
                                """!
                                @brief Transmisor asíncrono de telemetría de sensores.
                                @details Empaqueta el Objeto de Dominio (DTO) del sensor dentro del protocolo 
                                         R2Pilot y lo emite hacia el cliente móvil por WebSockets.
                                @param sensor_obj Objeto instanciado de la clase SensorEnvelope.
                                @return None
                                """
                                data_payload = StreamRespPayload(
                                    success=True, code=StatusCode.OK, resp_type=RespType.STREAM_RESP,
                                    stream_data=sensor_obj
                                )
                                await self._send_msg(RobotMessage(header=resp_header, payload=data_payload), send_callback)
                                
                            def ros2_sensor_callback(sensor_obj: Any) -> None:
                                """!
                                @brief Clausura inyectable para el suscriptor de tópicos de ROS 2.
                                @details Recibe los objetos de alta frecuencia (10Hz) del nodo de ROS 2 
                                         y delega su transmisión al hilo asíncrono de red de forma segura.
                                @param sensor_obj Instancia orientada a objetos con los datos del sensor.
                                @return None
                                """
                                asyncio.run_coroutine_threadsafe(_send_sensor_data(sensor_obj), main_loop)

                            success = self.ros_node.start_sensor_stream(topic, ros2_sensor_callback)
                            
                            resp_payload = StreamRespPayload(
                                success=success, 
                                code=StatusCode.OK if success else StatusCode.INTERNAL_ERROR, 
                                resp_type=RespType.STREAM_RESP,
                                details="Stream iniciado." if success else "Fallo al iniciar."
                            )
                    else:
                         resp_payload = StreamRespPayload(
                             success=False, code=StatusCode.INTERNAL_ERROR, resp_type=RespType.STREAM_RESP,
                             details="Backend no disponible."
                         )
                else:
                    resp_payload = StreamRespPayload(
                        success=False, code=StatusCode.NOT_ALLOWED, resp_type=RespType.STREAM_RESP,
                        details=f"Recurso {stream_payload.resource} no implementado."
                    )

            case MsgType.STOP_STREAM_REQ:
                stop_stream_payload = cast(StopStreamReqPayload, msg.payload)
                topic = getattr(stop_stream_payload, 'topic', 'Desconocido')
                self.logger.info(f"[STREAM] Solicitud de parada: {stop_stream_payload.resource} - {topic}")
                
                if stop_stream_payload.resource.upper() == Resource.SENSORS and self.ros_node:
                    if topic != 'Desconocido':
                        self.ros_node.stop_sensor_stream(topic)
                
                resp_payload = GenericRespPayload(
                    success=True, code=StatusCode.OK, resp_type=RespType.STOP_STREAM_RESP
                )

            case MsgType.ACK:
                self.logger.debug(f"[ACK] Cliente confirmó recepción del msg_id: {req_msg_id}")

            case _:
                self.logger.warning(f"[ROUTER] Comando {msg_type} recibido pero no implementado.")

        # =====================================================================
        # COMMIT DE TRANSICIÓN Y ENVÍO FINAL
        # =====================================================================
        if resp_payload:
            resp_msg = RobotMessage(header=resp_header, payload=resp_payload)

            self.state_machine.commit_transition(msg, resp_msg)
            await self._send_msg(resp_msg, send_callback)
            
            if msg_type == MsgType.COMMAND_REQ:
                cmd_payload = cast(CommandReqPayload, msg.payload)
                if cmd_payload.action in [Action.END, Action.REBOOT, Action.SHUTDOWN]:
                    is_success = getattr(resp_payload, 'success', False)
                    if is_success:
                        self.logger.info("[ROUTER] Cerrando el túnel de red de forma controlada...")
                        await close_callback()

    # =========================================================================
    # HELPERS INTERNOS
    # =========================================================================

    async def _send_msg(self, msg: RobotMessage, send_callback: Any) -> None:
        """! 
        @brief Transforma el modelo en JSON y lo dispara a la red.
        @details Delega la serialización al codec y ejecuta la clausura asíncrona inyectada
                 por el servidor físico.
        @param msg Objeto tipado que contiene la respuesta o notificación a enviar.
        @param send_callback Clausura asíncrona de envío (inyectada).
        @return None 
        """
        json_str = self.codec.encode(msg)
        await send_callback(json_str)

    def _build_error_msg(self, code: int, description: str, req_msg_id: int, session_id: str = "") -> RobotMessage:
        """! 
        @brief Constructor rápido de mensajes de error de protocolo estandarizados.
        @details Utilidad interna para evitar repetición de código al rechazar peticiones mal formadas
                 o denegadas por la máquina de estados.
        @param code Código de error HTTP-like (ej. 400, 403, 500).
        @param description Texto explicativo corto del motivo del fallo.
        @param req_msg_id ID del mensaje original que provocó el error (para trazar respuestas).
        @param session_id Identificador de la sesión afectada (opcional).
        @return Instancia de RobotMessage lista para ser codificada y enviada.
        """
        header = MessageHeader(msg_id=req_msg_id, type=MsgType.PROTOCOL_ERROR, session_id=session_id)
        payload = ProtocolErrorPayload(error_code=code, description=description)
        return RobotMessage(header=header, payload=payload)