"""
state_machine.py
El Semáforo del Protocolo.
Evalúa las transiciones sin ejecutarlas (can_transition) y 
las consolida solo si hay éxito (commit_transition).
"""

import logging
from typing import cast # <-- ¡NUEVO! Importamos cast
from app_tiago.utils.constants import (
    MsgType, Action, ControlEvent, 
    ServerState, MovementState, StatusCode
)
# <-- ¡NUEVO! Importamos los payloads específicos que vamos a leer
from app_tiago.protocol.models import RobotMessage, CommandReqPayload, ControlModeReqPayload

class ProtocolStateMachine:
    def __init__(self):
        self.logger = logging.getLogger("ProtocolStateMachine")
        self.global_state = ServerState.IDLE
        self.movement_state = MovementState.IDLE

    def client_connected(self):
        """Llamado cuando un WebSocket se ancla al servidor."""
        self.logger.info("Transición Global -> CONEXION_BACKEND (Cliente conectado)")
        self.global_state = ServerState.CONEXION_BACKEND
        
    def client_disconnected(self):
        """Llamado cuando el WebSocket se rompe o se cierra."""
        self.logger.info("Transición Global -> IDLE (Servidor esperando clientes)")
        self.global_state = ServerState.IDLE
        self.movement_state = MovementState.IDLE
    
    def can_transition(self, msg: RobotMessage) -> tuple[bool, int, str]:
        """
        SOLO EVALÚA: Comprueba si el mensaje es válido en el estado actual.
        No modifica ninguna variable de estado.
        """
        msg_type = msg.header.type

        # 0. MENSAJES TRANSVERSALES
        if msg_type in [MsgType.PING_REQ, MsgType.ACK]:
            return True, StatusCode.OK, ""
        
        # 1. BLOQUEO EN IDLE
        if self.global_state == ServerState.IDLE:
            return False, StatusCode.NOT_ALLOWED, "Servidor en reposo. No hay conexión física."

        # 2. ESTADO GLOBAL: CONEXION BACKEND
        if self.global_state == ServerState.CONEXION_BACKEND:
            if msg_type == MsgType.COMMAND_REQ:
                # SOLUCIÓN: Usamos cast para acceder a 'action' de forma segura
                cmd_payload = cast(CommandReqPayload, msg.payload)
                action = cmd_payload.action
                if action in [Action.CONNECT, Action.END]:
                    return True, StatusCode.OK, ""
                return False, StatusCode.NOT_ALLOWED, f"Acción '{action}' denegada. Primero envíe 'connect'."
            return False, StatusCode.NOT_ALLOWED, f"Mensaje '{msg_type}' denegado. Robot no conectado."

        # 2. ESTADO GLOBAL: SESION INICIADA
        elif self.global_state == ServerState.SESION_INICIADA:
            if msg_type == MsgType.COMMAND_REQ:
                # SOLUCIÓN: Usamos cast de nuevo
                cmd_payload = cast(CommandReqPayload, msg.payload)
                if cmd_payload.action == Action.DISCONNECT:
                    return True, StatusCode.OK, ""
                return False, StatusCode.NOT_ALLOWED, f"Acción '{cmd_payload.action}' denegada."

            elif msg_type in [MsgType.CONTROL_MODE_REQ, MsgType.CONTROL_REQ]:
                # SOLUCIÓN: Extraemos el evento de forma segura solo si es un CONTROL_MODE_REQ
                event = None
                if msg_type == MsgType.CONTROL_MODE_REQ:
                    cm_payload = cast(ControlModeReqPayload, msg.payload)
                    event = cm_payload.event

                # Estado actual: IDLE
                if self.movement_state == MovementState.IDLE:
                    if msg_type == MsgType.CONTROL_MODE_REQ and event == ControlEvent.START:
                        return True, StatusCode.OK, ""
                    return False, StatusCode.NOT_ALLOWED, "Comando de movimiento denegado. El estado es IDLE."
                
                # Estado actual: RECIBIENDO_INFO
                elif self.movement_state == MovementState.RECIBIENDO_INFO:
                    if msg_type == MsgType.CONTROL_REQ:
                        return True, StatusCode.OK, ""
                    elif msg_type == MsgType.CONTROL_MODE_REQ and event == ControlEvent.STOP:
                        return True, StatusCode.OK, ""
                    return False, StatusCode.NOT_ALLOWED, "Comando de movimiento denegado. El estado es RECIBIENDO_INFO."

            return False, StatusCode.NOT_ALLOWED, f"Mensaje '{msg_type}' no soportado en este estado."
            
        return False, StatusCode.INTERNAL_ERROR, "Estado interno desconocido."
    
    def commit_transition(self, req_msg: RobotMessage, resp_msg: RobotMessage):
        """
        CONSOLIDA EL CAMBIO: Lee la respuesta exacta que se va a enviar al cliente
        para sincronizar el estado interno del servidor.
        """
        # Si el servidor está mandando un aviso de desconexión (Watchdog)
        if resp_msg.header.type == MsgType.ASYNC_NOTIFY:
            if getattr(resp_msg.payload, "details", "") == "ROBOT_CONNECTION_LOST":
                self.trigger_session_reset()
            return

        # Para los RESP, extraemos el 'success' de forma segura
        success = getattr(resp_msg.payload, 'success', False)
        req_type = req_msg.header.type

        if req_type == MsgType.COMMAND_REQ:
            # SOLUCIÓN: Cast para acceder a 'action'
            cmd_req_payload = cast(CommandReqPayload, req_msg.payload)
            action = cmd_req_payload.action
            if success:
                if action == Action.CONNECT:
                    self.global_state = ServerState.SESION_INICIADA
                    self.movement_state = MovementState.IDLE
                    self.logger.info("Transición Global -> SESION_INICIADA")
                elif action == Action.DISCONNECT:
                    self.trigger_session_reset()
                elif action == Action.END:
                    self.logger.info("Transición -> FIN DEL PROTOCOLO")
                    
        elif req_type == MsgType.CONTROL_MODE_REQ:
            # SOLUCIÓN: Cast para acceder a 'event'
            cm_req_payload = cast(ControlModeReqPayload, req_msg.payload)
            event = cm_req_payload.event
            if success:
                if event == ControlEvent.START:
                    self.movement_state = MovementState.RECIBIENDO_INFO
                    self.logger.info("Transición Movimiento -> RECIBIENDO_INFO")
                elif event == ControlEvent.STOP:
                    self.movement_state = MovementState.IDLE
                    self.logger.info("Transición Movimiento -> IDLE")

        elif req_type == MsgType.CONTROL_REQ:
            if not success:
                self.movement_state = MovementState.IDLE
                self.logger.warning("Respuesta de Error en CONTROL_REQ. Vuelta a IDLE de emergencia.")

    def trigger_session_reset(self):
        self.logger.info("Limpiando sesión... Volviendo a CONEXION_BACKEND")
        self.global_state = ServerState.CONEXION_BACKEND
        self.movement_state = MovementState.IDLE