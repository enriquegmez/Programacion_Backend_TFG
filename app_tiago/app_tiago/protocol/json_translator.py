"""
message_factory_parser.py
Centraliza la codificación y decodificación de los mensajes.
Implementación estricta basada en el JSON Schema del Protocolo TIAGO.
"""

import json
import time
import logging
from typing import Dict, Any, Optional
from app_tiago.utils.constants import MsgType, StatusCode

class MessageCodec:
    def __init__(self):
        self.logger = logging.getLogger("MessageCodec")
        # El contador de mensajes asegura que el msg_id sea único y secuencial
        self._msg_id_counter = 0

    def _get_next_msg_id(self) -> int:
        """Genera el siguiente msg_id válido (entero >= 0)."""
        current_id = self._msg_id_counter
        self._msg_id_counter += 1
        return current_id

    # ==========================================
    # 1. PARSER (Entrada de red a Python)
    # ==========================================
    def decode(self, raw_string: str) -> Dict[str, Any]:
        """
        Convierte el string del WebSocket en un diccionario Python.
        Solo valida la sintaxis JSON. La estructura la validará el validator.py.
        """
        try:
            parsed_data = json.loads(raw_string)
            # Intentamos leer el tipo del header de forma segura para el log
            header = parsed_data.get("header", {})
            self.logger.debug(f"Mensaje decodificado. Tipo: {header.get('type', 'UNKNOWN')}")
            return parsed_data
            
        except json.JSONDecodeError as e:
            self.logger.error(f"Error crítico de formato JSON: {e}")
            # Si la sintaxis está rota, generamos la estructura de PROTOCOL_ERROR
            # que tu schema exige internamente para que el router lo maneje.
            return self._build_internal_error_dict(StatusCode.BAD_REQUEST, "Invalid JSON format")

    def _build_internal_error_dict(self, code: int, description: str) -> Dict[str, Any]:
        """Genera un diccionario de error interno compatible con tu schema."""
        return {
            "header": {
                "msg_id": self._get_next_msg_id(),
                "timestamp": time.time(),
                "type": MsgType.PROTOCOL_ERROR,
                "session_id": ""
            },
            "payload": {
                "error_code": code,
                "description": description
            }
        }

    # ==========================================
    # 2. BASE FACTORY (Motor interno de empaquetado)
    # ==========================================
    def _encode(self, msg_type: str, payload: Dict[str, Any], session_id: str = "") -> str:
        """
        Construye EL SOBRE. Garantiza que todos los mensajes salientes 
        cumplan la estructura base: header (con msg_id y timestamp automatizados) y payload.
        """
        message_dict = {
            "header": {
                "msg_id": self._get_next_msg_id(),
                "timestamp": time.time(),
                "type": msg_type,
                "session_id": session_id
            },
            "payload": payload
        }
        return json.dumps(message_dict)

    def _build_resp_base(self, resp_type: str, success: bool, code: int, 
                         session_id: str, extra_payload: Optional[Dict] = None, 
                         details: str = None) -> str:
        """
        Construye el payload base para los mensajes de tipo 'RESP'.
        Maneja la lógica estricta de tu schema (ej. si success es false, 'details' es obligatorio).
        """
        payload = {
            "success": success,
            "code": code,
            "resp_type": resp_type
        }
        
        if extra_payload:
            payload.update(extra_payload)
            
        # Regla de tu schema: "Si success es FALSE, obligamos a que haya un mensaje en details"
        if not success:
            payload["details"] = details if details else "Error desconocido"
        elif details:
            payload["details"] = details

        # En tu schema, todas las respuestas comparten el tipo de cabecera 'RESP'
        return self._encode(MsgType.RESP, payload, session_id)

    # ==========================================
    # 3. CONSTRUCTORES PÚBLICOS (La API del Factory)
    # ==========================================

    def build_connect_success(self, current_session_id_str: str, assigned_session_id_int: int) -> str:
        """
        Respuesta específica de éxito para la acción 'connect'.
        Según tu schema, si resp_type='COMMAND_RESP' y success=True, 
        el payload debe incluir 'session_id' (como integer).
        """
        extra_payload = {
            "session_id": assigned_session_id_int
        }
        return self._build_resp_base(
            resp_type=MsgType.COMMAND_RESP,
            success=True,
            code=StatusCode.OK,
            session_id=current_session_id_str, # String para el header
            extra_payload=extra_payload
        )

    def build_protocol_error(self, error_code: int, description: str, session_id: str = "") -> str:
        """
        Construye el mensaje principal de PROTOCOL_ERROR.
        Tu schema exige que el header sea type='PROTOCOL_ERROR' y el payload lleve 'error_code' y 'description'.
        """
        payload = {
            "error_code": error_code,
            "description": description
        }
        return self._encode(MsgType.PROTOCOL_ERROR, payload, session_id)

    # TODO: En el futuro se añadirán aquí los métodos build_query_resp, build_stream_resp, etc.