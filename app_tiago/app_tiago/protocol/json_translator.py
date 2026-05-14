"""
json_translator.py
Centraliza la codificación y decodificación de los mensajes.
Traduce entre JSON puro (para la red) y objetos RobotMessage (para el backend).
"""

import json
import time
import logging
import dataclasses
from typing import Any

from app_tiago.utils.constants import MsgType, StatusCode, RespType
from app_tiago.protocol.models import (
    RobotMessage, MessageHeader, CommandReqPayload, QueryReqPayload,
    ActionReqPayload, ControlModeReqPayload, ControlReqPayload, ControlData,
    StreamReqPayload, StopStreamReqPayload, AsyncNotifyPayload,
    ProtocolErrorPayload, EmptyPayload, QueryRespPayload,
    ActionFeedbackPayload, StreamRespPayload, GenericRespPayload
)

from app_tiago.protocol.validator import ProtocolValidator

class MessageCodec:
    def __init__(self):
        self.logger = logging.getLogger("MessageCodec")
        self._msg_id_counter = 0
        self.validator = ProtocolValidator()

    def _get_next_msg_id(self) -> int:
        current_id = self._msg_id_counter
        self._msg_id_counter += 1
        return current_id

    # ==========================================
    # 1. PARSER (Entrada de red a Python)
    # ==========================================
    def decode(self, raw_string: str) -> RobotMessage:
        """
        Convierte el string del WebSocket en un objeto RobotMessage estructurado.
        Si hay un error JSON, devuelve un RobotMessage de tipo PROTOCOL_ERROR.
        """
        try:
            parsed_dict = json.loads(raw_string)
        except json.JSONDecodeError as e:
            self.logger.error(f"Error crítico de formato JSON: {e}")
            return self._build_internal_error_msg(StatusCode.BAD_REQUEST, "Invalid JSON format")

        # --- CAPA DE VALIDACIÓN DEL SCHEMA ---
        is_valid, error_desc = self.validator.validate_message(parsed_dict)
        if not is_valid:
            # Si el JSON no cumple el esquema, devolvemos un PROTOCOL_ERROR directamente
            return self._build_internal_error_msg(StatusCode.BAD_REQUEST, error_desc)
        # -------------------------------------

        # 1. Extraer Header
        header_data = parsed_dict.get("header", {})
        msg_type = header_data.get("type", "UNKNOWN")
        
        header = MessageHeader(
            msg_id=header_data.get("msg_id", -1),
            type=msg_type,
            session_id=header_data.get("session_id", ""),
            timestamp=header_data.get("timestamp", time.time())
        )

        # 2. Extraer Payload según el tipo de mensaje
        raw_payload = parsed_dict.get("payload", {})

        payload: Any
        
        try:
            if msg_type == MsgType.COMMAND_REQ:
                payload = CommandReqPayload(**raw_payload)
            elif msg_type == MsgType.QUERY_REQ:
                payload = QueryReqPayload(**raw_payload)
            elif msg_type in [MsgType.ACTION_REQ, MsgType.STOP_ACTION_REQ]:
                payload = ActionReqPayload(**raw_payload)
            elif msg_type == MsgType.CONTROL_MODE_REQ:
                payload = ControlModeReqPayload(**raw_payload)
            elif msg_type == MsgType.CONTROL_REQ:
                data_dict = raw_payload.get("data", {})
                payload = ControlReqPayload(data=ControlData(**data_dict))
            elif msg_type == MsgType.STREAM_REQ:
                payload = StreamReqPayload(**raw_payload)
            elif msg_type == MsgType.STOP_STREAM_REQ:
                payload = StopStreamReqPayload(**raw_payload)
            elif msg_type == MsgType.ASYNC_NOTIFY:
                payload = AsyncNotifyPayload(**raw_payload)
            elif msg_type == MsgType.PROTOCOL_ERROR:
                payload = ProtocolErrorPayload(**raw_payload)
            elif msg_type in [MsgType.PING_REQ, MsgType.ACK]:
                payload = EmptyPayload()
            elif msg_type == MsgType.RESP:
                resp_type = raw_payload.get("resp_type")
                if resp_type == RespType.QUERY_RESP:
                    payload = QueryRespPayload(**raw_payload)
                elif resp_type in [RespType.ACTION_FEEDBACK, RespType.STOP_ACTION_FEEDBACK]:
                    payload = ActionFeedbackPayload(**raw_payload)
                elif resp_type == RespType.STREAM_RESP:
                    payload = StreamRespPayload(**raw_payload)
                else:
                    payload = GenericRespPayload(**raw_payload)
            else:
                self.logger.warning(f"Tipo de mensaje desconocido: {msg_type}")
                return self._build_internal_error_msg(StatusCode.BAD_REQUEST, f"Unknown message type: {msg_type}")
                
        except TypeError as e:
            self.logger.error(f"Faltan campos en el payload o son incorrectos: {e}")
            return self._build_internal_error_msg(StatusCode.BAD_REQUEST, "Malformed payload")

        return RobotMessage(header=header, payload=payload)

    def _build_internal_error_msg(self, code: int, description: str) -> RobotMessage:
        """Genera un objeto RobotMessage de error interno para que el router lo gestione."""
        header = MessageHeader(
            msg_id=-1, # Se asignará uno real al enviar
            type=MsgType.PROTOCOL_ERROR,
            session_id=""
        )
        payload = ProtocolErrorPayload(error_code=code, description=description)
        return RobotMessage(header=header, payload=payload)

    # ==========================================
    # 2. ENCODER (De Python a String JSON)
    # ==========================================
    def encode(self, message: RobotMessage) -> str:
        """
        Toma un objeto RobotMessage, inyecta los datos temporales en la cabecera
        y lo convierte en un string JSON limpio y válido.
        """
        # 1. Autocompletar datos del sistema en la cabecera antes de enviar
        # Si el msg_id es menor o igual a 0, generamos uno nuevo (Notificaciones, Iniciativa del Server)
        # Si ya tiene un ID válido (>0), lo respetamos (Eco de Respuestas)
        if message.header.msg_id <= 0:
            message.header.msg_id = self._get_next_msg_id()
            
        message.header.timestamp = time.time()

        # 2. Convertir el objeto dataclass completo a diccionario
        raw_dict = dataclasses.asdict(message)

        # 3. Limpiar los valores nulos (None)
        # Esto asegura que si un campo opcional no se usa, no aparezca en el JSON como "null",
        # ahorrando ancho de banda y cumpliendo estrictamente con el JSON Schema.
        clean_dict = self._remove_none_values(raw_dict)

        return json.dumps(clean_dict)

    def _remove_none_values(self, data: Any) -> Any:
        """
        Función recursiva para eliminar todas las claves cuyo valor sea None.
        """
        if isinstance(data, dict):
            return {k: self._remove_none_values(v) for k, v in data.items() if v is not None}
        elif isinstance(data, list):
            return [self._remove_none_values(v) for v in data if v is not None]
        return data