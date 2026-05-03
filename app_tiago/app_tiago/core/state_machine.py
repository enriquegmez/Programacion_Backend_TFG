"""
state_machine.py
El Semáforo del Protocolo (VERSIÓN ITERATIVA - REESTRUCTURADA).
Evalúa las transiciones basándose primero en el estado actual.
"""

import logging
from app_tiago.utils.constants import (
    MsgType, Action, ControlEvent, 
    ServerState, MovementState, StatusCode
)
from app_tiago.protocol.models import RobotMessage

class ProtocolStateMachine:
    def __init__(self):
        self.logger = logging.getLogger("ProtocolStateMachine")
        self.global_state = ServerState.CONEXION_BACKEND
        self.movement_state = MovementState.IDLE

    def check_and_transition(self, msg: RobotMessage) -> tuple[bool, int, str]:
        """
        Evalúa el mensaje entrante según la jerarquía de estados.
        Retorna: (is_valid, status_code, error_description)
        """
        # Primero, extraemos el tipo de mensaje para decidir qué reglas aplicar
        msg_type = msg.header.type

        # ==========================================
        # 0. MENSAJES TRANSVERSALES (Siempre permitidos)
        # ==========================================
        if msg_type in [MsgType.PING_REQ, MsgType.ACK]:
            return True, StatusCode.OK, ""

        # ==========================================
        # 1. ESTADO GLOBAL: CONEXION BACKEND
        # ==========================================
        if self.global_state == ServerState.CONEXION_BACKEND:
            if msg_type == MsgType.COMMAND_REQ:
                action = msg.payload.action
                
                if action == Action.CONNECT:
                    self.global_state = ServerState.SESION_INICIADA
                    self.movement_state = MovementState.IDLE
                    self.logger.info("Transición Global -> SESION_INICIADA")
                    return True, StatusCode.OK, ""
                    
                elif action == Action.END:
                    self.logger.info("Transición -> FIN DEL PROTOCOLO")
                    return True, StatusCode.OK, ""
                
                return False, StatusCode.NOT_ALLOWED, f"Acción '{action}' denegada. Primero envíe 'connect'."
            
            return False, StatusCode.NOT_ALLOWED, f"Mensaje '{msg_type}' denegado. Robot no conectado."

        # ==========================================
        # 2. ESTADO GLOBAL: SESION INICIADA
        # ==========================================
        elif self.global_state == ServerState.SESION_INICIADA:
            
            # 2.1 Mensajes globales de la sesión
            if msg_type == MsgType.COMMAND_REQ:
                if msg.payload.action == Action.DISCONNECT:
                    self.trigger_session_reset()
                    return True, StatusCode.OK, ""
                else:
                    return False, StatusCode.NOT_ALLOWED, f"Acción '{msg.payload.action}' denegada."

            # 2.2 Mensajes de la región: MOVIMIENTO
            elif msg_type in [MsgType.CONTROL_MODE_REQ, MsgType.CONTROL_REQ]:
                
                # Estado actual: IDLE
                if self.movement_state == MovementState.IDLE:
                    if msg_type == MsgType.CONTROL_MODE_REQ and msg.payload.event == ControlEvent.START:
                        self.movement_state = MovementState.RECIBIENDO_INFO
                        self.logger.info("Transición Movimiento -> RECIBIENDO_INFO")
                        return True, StatusCode.OK, ""
                    else:
                        # TU SUGERENCIA: Cualquier otro mensaje de movimiento aquí, se rechaza.
                        return False, StatusCode.NOT_ALLOWED, "Comando de movimiento denegado. El estado es IDLE."
                
                # Estado actual: RECIBIENDO_INFO
                elif self.movement_state == MovementState.RECIBIENDO_INFO:
                    if msg_type == MsgType.CONTROL_REQ:
                        return True, StatusCode.OK, ""
                    elif msg_type == MsgType.CONTROL_MODE_REQ and msg.payload.event == ControlEvent.STOP:
                        self.movement_state = MovementState.IDLE
                        self.logger.info("Transición Movimiento -> IDLE")
                        return True, StatusCode.OK, ""
                    else:
                        # TU SUGERENCIA: Si mandan un START u otra cosa rara aquí, se rechaza.
                        return False, StatusCode.NOT_ALLOWED, "Comando de movimiento denegado. El estado es RECIBIENDO_INFO."

            # Si el mensaje no es ni transversal, ni de sesión, ni de movimiento (o no está implementado aún)
            return False, StatusCode.NOT_ALLOWED, f"Mensaje '{msg_type}' no soportado en este estado."

    # ==========================================
    # EVENTOS GLOBALES
    # ==========================================
    def trigger_session_reset(self):
        """Limpia la sesión y vuelve a Conexion Backend."""
        self.logger.info("Limpiando sesión... Volviendo a CONEXION_BACKEND")
        self.global_state = ServerState.CONEXION_BACKEND
        self.movement_state = MovementState.IDLE