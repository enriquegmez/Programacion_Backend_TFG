## @file state_machine.py
#  @brief La máquina de estados del Protocolo (FSM).
#  @details Implementa el patrón Máquina de Estados Finitos para garantizar que 
#           los comandos del protocolo R2Pilot se ejecuten en un orden seguro y en sincronía con el cliente.
#           Evalúa las transiciones sin ejecutarlas (can_transition) y 
#           las consolida solo si hay éxito confirmado (commit_transition).
#  @author Enrique Gómez
#  @date 2026

import logging
from typing import cast, Optional, Tuple
from r2pilot_server.utils.constants import (
    MonitorState, MsgType, Action, ControlEvent, 
    ServerState, MovementState, StatusCode, Resource
)
from r2pilot_server.protocol.models import (
    RobotMessage, CommandReqPayload, ActionReqPayload, 
    ControlModeReqPayload, StopActionReqPayload, StreamReqPayload, 
    StopStreamReqPayload, QueryReqPayload
)

class ProtocolStateMachine:
    """!
    @brief Máquina de estados del protocolo de comunicación R2Pilot.
    @details Mantiene tres submáquinas paralelas (Global, Movimiento y Monitorización)
             para permitir operaciones concurrentes seguras (ej. teleoperar mientras
             se hace streaming de vídeo), evitando colisiones de comandos.
    """

    def __init__(self) -> None:
        """!
        @brief Inicializa los estados lógicos por defecto y memorias de la máquina.
        """
        self.logger = logging.getLogger("ProtocolStateMachine")
        
        ## Estado maestro de la conexión de red
        self.global_state = ServerState.IDLE
        ## Estado concurrente A: Tracción y manipuladores
        self.movement_state = MovementState.IDLE
        ## Estado concurrente B: Flujos de datos de alta frecuencia
        self.monitor_state = MonitorState.IDLE
        
        ## Puntero al payload de la acción de ROS 2 actualmente en curso
        self.current_action: Optional[ActionReqPayload] = None
        
        ## Conjunto (Set) de strings para rastrear los tópicos de streaming activos
        self.active_streams: set[str] = set()

    def client_connected(self) -> None:
        """!
        @brief Evento disparado cuando un socket físico se ancla al servidor.
        @details Se pasa al estado de conexión para permitir el inicio de la comunicación.
        """
        self.logger.info("[FSM] Transición Global -> CONEXION_BACKEND (Cliente físico conectado)")
        self.global_state = ServerState.CONEXION_BACKEND
        
    def client_disconnected(self) -> None:
        """!
        @brief Evento disparado cuando la conexión física se destruye.
        @details Elimina toda la memoria de estados y variables para el próximo cliente.
        """
        self.logger.info("[FSM] Transición Global -> IDLE (Servidor en reposo esperando clientes)")
        self.global_state = ServerState.IDLE
        self.movement_state = MovementState.IDLE
        self.monitor_state = MonitorState.IDLE
        self.current_action = None
        self.active_streams.clear()
    
    def can_transition(self, msg: RobotMessage) -> Tuple[bool, int, str]:
        """!
        @brief Validador de mensajes entrantes.
        @details Comprueba si el mensaje entrante es legal según las reglas 
                 del estado actual. Actúa como un semáforo antes de enviar nada a ROS 2.
                 No modifica ninguna variable interna de estado.
        @param msg El mensaje decodificado recibido del cliente.
        @return Tupla (is_valid, HTTP_status_code, error_description).
        """
        msg_type = msg.header.type

        # 0. MENSAJES TRANSVERSALES (Heartbeat y ACKs)
        if msg_type in [MsgType.PING_REQ, MsgType.ACK]:
            return True, StatusCode.OK, ""
        
        # 1. BLOQUEO ABSOLUTO EN IDLE
        if self.global_state == ServerState.IDLE:
            return False, StatusCode.NOT_ALLOWED, "Servidor en reposo. No hay conexión física."

        # 2. ESTADO GLOBAL: CONEXIÓN BACKEND (Negociación inicial y Lobby)
        if self.global_state == ServerState.CONEXION_BACKEND:
            if msg_type == MsgType.COMMAND_REQ:
                cmd_payload = cast(CommandReqPayload, msg.payload)
                action = cmd_payload.action
                if action in [Action.CONNECT, Action.END, Action.CHANGE_VARS, Action.REBOOT, Action.SHUTDOWN]:
                    return True, StatusCode.OK, ""
                return False, StatusCode.NOT_ALLOWED, f"Acción '{action}' denegada. Primero envíe 'connect'."
            
            # Consultas permitidas desde la sala de espera
            if msg_type == MsgType.QUERY_REQ:
                query_payload = cast(QueryReqPayload, msg.payload)
                if query_payload.resource_type == Resource.HOST_INFO:
                    return True, StatusCode.OK, ""
                return False, StatusCode.NOT_ALLOWED, f"La consulta '{query_payload.resource_type}' no está permitida en la sala de espera."

            return False, StatusCode.NOT_ALLOWED, f"Mensaje '{msg_type}' denegado. Sesión no iniciada."

        # 3. ESTADO GLOBAL: SESIÓN INICIADA (Protocolo activo)
        elif self.global_state == ServerState.SESION_INICIADA:
            
            # --- SUBMÁQUINA DE SESIÓN Y COMANDOS ---
            if msg_type == MsgType.COMMAND_REQ:
                cmd_payload = cast(CommandReqPayload, msg.payload)
                if cmd_payload.action == Action.DISCONNECT:
                    return True, StatusCode.OK, ""
                return False, StatusCode.NOT_ALLOWED, f"Acción '{cmd_payload.action}' denegada."
            
            # --- TRANSVERSAL: CONSULTAS ESTÁTICAS ---
            elif msg_type == MsgType.QUERY_REQ:
                return True, StatusCode.OK, ""

            # --- SUBMÁQUINA CONCURRENTE A: MOVIMIENTO ---
            elif msg_type in [MsgType.ACTION_REQ, MsgType.STOP_ACTION_REQ, MsgType.CONTROL_MODE_REQ, MsgType.CONTROL_REQ]:
                event = None
                if msg_type == MsgType.CONTROL_MODE_REQ:
                    cm_payload = cast(ControlModeReqPayload, msg.payload)
                    event = cm_payload.event

                if self.movement_state == MovementState.IDLE:
                    if msg_type == MsgType.CONTROL_MODE_REQ and event == ControlEvent.START:
                        return True, StatusCode.OK, ""
                    elif msg_type == MsgType.ACTION_REQ:
                        if self.current_action is not None:
                            return False, StatusCode.NOT_ALLOWED, "Error de estado: acción en memoria pero estado IDLE."
                        return True, StatusCode.OK, ""
                    return False, StatusCode.NOT_ALLOWED, "Comando de movimiento denegado. El estado es IDLE."
                
                elif self.movement_state == MovementState.RECIBIENDO_INFO:
                    if msg_type == MsgType.CONTROL_REQ:
                        return True, StatusCode.OK, ""
                    elif msg_type == MsgType.CONTROL_MODE_REQ and event == ControlEvent.STOP:
                        return True, StatusCode.OK, ""
                    elif msg_type == MsgType.ACTION_REQ:
                        return False, StatusCode.NOT_ALLOWED, "Comando denegado. El servidor está en modo de teleoperación."
                    return False, StatusCode.NOT_ALLOWED, "Comando de movimiento denegado. El estado es RECIBIENDO_INFO."
                
                elif self.movement_state == MovementState.EJECUTANDO_ACCION:
                    if msg_type == MsgType.STOP_ACTION_REQ:
                        action_payload = cast(StopActionReqPayload, msg.payload)
                        if self.current_action is None:
                            return False, StatusCode.NOT_ALLOWED, "No se puede detener: no hay ninguna acción en curso."
                        if (action_payload.type != self.current_action.type or
                                action_payload.target != self.current_action.target):
                            return False, StatusCode.NOT_ALLOWED, "La acción pedida no coincide con la acción en curso."
                        return True, StatusCode.OK, ""
                    
                    elif msg_type == MsgType.ACTION_REQ:
                        return False, StatusCode.NOT_ALLOWED, "Comando denegado. Ya hay una acción en ejecución."
                    return False, StatusCode.NOT_ALLOWED, "Comando denegado. Se está ejecutando una acción compleja."

            # --- SUBMÁQUINA CONCURRENTE B: MONITORIZACIÓN (SENSORES/VÍDEO) ---
            elif msg_type in [MsgType.STREAM_REQ, MsgType.STOP_STREAM_REQ]:
                if msg_type == MsgType.STREAM_REQ:
                    return True, StatusCode.OK, ""
                
                elif msg_type == MsgType.STOP_STREAM_REQ:
                    if self.monitor_state == MonitorState.ENVIANDO_STREAM:
                        return True, StatusCode.OK, ""
                    return False, StatusCode.NOT_ALLOWED, "Petición denegada: No hay ningún stream activo para detener."

            return False, StatusCode.NOT_ALLOWED, f"Mensaje '{msg_type}' no soportado en este estado."
            
        return False, StatusCode.INTERNAL_ERROR, "Estado interno de la máquina desconocido."
    
    def commit_transition(self, req_msg: RobotMessage, resp_msg: RobotMessage) -> None:
        """!
        @brief Consolida el cambio de estado de forma reactiva.
        @details Lee la respuesta exacta que el director acaba de generar (éxito o error). 
                 Solo avanza el estado interno del servidor si ROS 2 o el sistema aceptaron la orden.
        @param req_msg Mensaje original de petición enviado por el cliente.
        @param resp_msg Mensaje de respuesta consolidado que se va a enviar al cliente.
        """
        # Excepción por Watchdog
        if resp_msg.header.type == MsgType.ASYNC_NOTIFY:
            if getattr(resp_msg.payload, "details", "") == "ROBOT_CONNECTION_LOST":
                self.trigger_session_reset()
            return

        success = getattr(resp_msg.payload, 'success', False)
        req_type = req_msg.header.type

        if req_type == MsgType.COMMAND_REQ:
            cmd_req_payload = cast(CommandReqPayload, req_msg.payload)
            action = cmd_req_payload.action
            if success:
                if action == Action.CONNECT:
                    self.global_state = ServerState.SESION_INICIADA
                    self.movement_state = MovementState.IDLE
                    self.monitor_state = MonitorState.IDLE
                    self.logger.info("[FSM] Transición Global -> SESION_INICIADA")
                elif action == Action.DISCONNECT:
                    self.trigger_session_reset()
                elif action == Action.END:
                    self.logger.info("[FSM] Transición -> FIN DEL PROTOCOLO (Cierre solicitado)")
                elif action in [Action.REBOOT, Action.SHUTDOWN]:
                    self.logger.info(f"[FSM] El sistema ha ordenado: {action.upper()}. Preparando desconexión inminente...")
                    self.trigger_session_reset()
                    
        # ==========================================
        # TRANSICIONES CONSOLIDADAS DE MOVIMIENTO
        # ==========================================
        elif req_type == MsgType.CONTROL_MODE_REQ:
            cm_req_payload = cast(ControlModeReqPayload, req_msg.payload)
            event = cm_req_payload.event
            mode_type = cm_req_payload.type 
            if success:
                if event == ControlEvent.START:
                    self.movement_state = MovementState.RECIBIENDO_INFO
                    self.logger.info(f"[FSM] Transición Movimiento -> RECIBIENDO_INFO (Modo: {mode_type})")
                elif event == ControlEvent.STOP:
                    self.movement_state = MovementState.IDLE
                    self.logger.info(f"[FSM] Transición Movimiento -> IDLE (Fin de {mode_type})")

        elif req_type == MsgType.CONTROL_REQ:
            if not success:
                self.movement_state = MovementState.IDLE
                self.logger.warning("[FSM] Error en CONTROL_REQ. Vuelta a IDLE de emergencia.")

        elif req_type == MsgType.ACTION_REQ:
            if success:
                action_payload = cast(ActionReqPayload, req_msg.payload)
                self.current_action = action_payload
                done_exec = getattr(resp_msg.payload, 'done_exec', False)
                
                if done_exec:
                    self.movement_state = MovementState.IDLE
                    self.current_action = None
                    self.logger.info("[FSM] Transición Movimiento -> IDLE (Acción terminada)")
                else:
                    if self.movement_state != MovementState.EJECUTANDO_ACCION:
                        self.movement_state = MovementState.EJECUTANDO_ACCION
                        self.logger.info("[FSM] Transición Movimiento -> EJECUTANDO_ACCION")
            else:
                if self.movement_state == MovementState.EJECUTANDO_ACCION:
                    self.movement_state = MovementState.IDLE
                    self.current_action = None
                    self.logger.warning("[FSM] Error ejecutando la acción. Vuelta a IDLE de emergencia.")

        elif req_type == MsgType.STOP_ACTION_REQ:
            if success:
                self.movement_state = MovementState.IDLE
                self.current_action = None
                self.logger.info("[FSM] Transición Movimiento -> IDLE (Acción abortada)")

        elif req_type == MsgType.QUERY_REQ:
            pass
        
        # ==========================================
        # TRANSICIONES CONSOLIDADAS DE MONITORIZACIÓN
        # ==========================================
        elif req_type == MsgType.STREAM_REQ:
            if success:
                stream_payload = cast(StreamReqPayload, req_msg.payload)
                new_stream_id = stream_payload.topic if stream_payload.topic else stream_payload.resource
                
                self.active_streams.add(new_stream_id)
                self.monitor_state = MonitorState.ENVIANDO_STREAM
                self.logger.info(f"[FSM] Monitorización -> ENVIANDO_STREAM (Añadido: {new_stream_id}. Activos: {len(self.active_streams)})")
                
        elif req_type == MsgType.STOP_STREAM_REQ:
            if success:
                stop_payload = cast(StopStreamReqPayload, req_msg.payload)
                stopped_stream_id: str = stop_payload.topic if stop_payload.topic else stop_payload.resource
                
                self.active_streams.discard(stopped_stream_id)
                
                if len(self.active_streams) == 0:
                    self.monitor_state = MonitorState.IDLE
                    self.logger.info("[FSM] Transición Monitorización -> IDLE (0 streams activos)")
                else:
                    self.logger.info(f"[FSM] Monitorización -> Sigue en ENVIANDO_STREAM (Restantes: {len(self.active_streams)})")

    def trigger_session_reset(self) -> None:
        """!
        @brief Fuerza un retroceso a la sala de espera.
        @details Útil tras desconexiones de ROS 2, paradas de emergencia o apagados de hardware.
        """
        self.logger.info("[FSM] Limpiando memoria de sesión... Volviendo a CONEXION_BACKEND")
        self.global_state = ServerState.CONEXION_BACKEND
        self.movement_state = MovementState.IDLE
        self.monitor_state = MonitorState.IDLE
        self.current_action = None
        self.active_streams.clear()