## @file message_codec.py
#  @brief Traductor central y capa de serialización del protocolo.
#  @details Actúa como frontera entre el mundo exterior (Strings JSON) y el núcleo
#           del servidor (Objetos de Python / Dataclasses). Aplica la validación estricta
#           del JSON Schema antes de instanciar cualquier objeto.
#  @author Enrique Gómez
#  @date 2026

import json
import time
import logging
import dataclasses
from typing import Any

from r2pilot_server.utils.constants import MsgType, StatusCode, RespType
from r2pilot_server.protocol.models import (
    RobotMessage, MessageHeader, CommandReqPayload, QueryReqPayload,
    ActionReqPayload, ControlModeReqPayload, ControlReqPayload, ControlData,
    StreamReqPayload, StopStreamReqPayload, AsyncNotifyPayload,
    ProtocolErrorPayload, EmptyPayload, QueryRespPayload,
    ActionFeedbackPayload, StreamRespPayload, GenericRespPayload, StopActionReqPayload
)

# Enlace con la validación estricta del contrato
from r2pilot_server.protocol.validator import ProtocolValidator


class MessageCodec:
    """!
    @brief Motor de codificación y decodificación asimétrica.
    @details Se asegura de que ningún paquete mal formado entre en la máquina
             de estados y garantiza que todas las respuestas del servidor salgan 
             comprimidas (sin nulos) y con un ID válido.
    """

    def __init__(self) -> None:
        """!
        @brief Inicializa el traductor, el validador de esquemas y el contador de paquetes.
        """
        self.logger = logging.getLogger("MessageCodec")
        ## Contador incremental para garantizar un msg_id único en peticiones del servidor
        self._msg_id_counter = 0
        ## Instancia del validador de JSON Schema (Draft-07)
        self.validator = ProtocolValidator()

    def _get_next_msg_id(self) -> int:
        """!
        @brief Genera un identificador de paquete único y autoincremental.
        @return Entero que representa el nuevo ID del mensaje.
        """
        current_id = self._msg_id_counter
        self._msg_id_counter += 1
        return current_id

    # =========================================================================
    # PARSER (Entrada de red a Objetos de Dominio)
    # =========================================================================

    def decode(self, raw_string: str) -> RobotMessage:
        """!
        @brief Convierte la cadena cruda del WebSocket (json) en un objeto de dominio tipado.
        @details Supera dos filtros: 1) Que el String sea un JSON válido. 2) Que el JSON 
                 cumpla matemáticamente con el Schema del protocolo. Si falla, encapsula 
                 el error en un RobotMessage de tipo PROTOCOL_ERROR.
        @param raw_string Trama de texto capturada de la red.
        @return Objeto RobotMessage listo para ser evaluado por la máquina de estados.
        """
        # 1. INTEGRIDAD DE TEXTO A DICCIONARIO
        try:
            parsed_dict = json.loads(raw_string)
        except json.JSONDecodeError as e:
            self.logger.error(f"[CODEC] Error crítico de parseo JSON: {e}")
            return self._build_internal_error_msg(StatusCode.BAD_REQUEST, "Formato JSON inválido.")

        # 2. CAPA DE VALIDACIÓN DEL SCHEMA (Contrato)
        is_valid, error_desc = self.validator.validate_message(parsed_dict)
        if not is_valid:
            # Bloqueamos el paquete en la frontera y devolvemos el fallo
            return self._build_internal_error_msg(StatusCode.BAD_REQUEST, error_desc)

        # 3. EXTRACCIÓN DE METADATOS (Header)
        header_data = parsed_dict.get("header", {})
        msg_type = header_data.get("type", "UNKNOWN")
        
        header = MessageHeader(
            msg_id=header_data.get("msg_id", -1),
            type=msg_type,
            session_id=header_data.get("session_id", ""),
            timestamp=header_data.get("timestamp", time.time())
        )

        # 4. INSTANCIACIÓN POLIMÓRFICA (Payload)
        raw_payload = parsed_dict.get("payload", {})
        payload: Any
        
        try:
            if msg_type == MsgType.COMMAND_REQ:
                payload = CommandReqPayload(**raw_payload)
            elif msg_type == MsgType.QUERY_REQ:
                payload = QueryReqPayload(**raw_payload)
            elif msg_type == MsgType.ACTION_REQ:
                payload = ActionReqPayload(**raw_payload)
            elif msg_type == MsgType.STOP_ACTION_REQ:
                payload = StopActionReqPayload(**raw_payload)  
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
                elif resp_type == RespType.ACTION_FEEDBACK:
                    payload = ActionFeedbackPayload(**raw_payload)
                elif resp_type == RespType.STREAM_RESP:
                    payload = StreamRespPayload(**raw_payload)
                else:
                    payload = GenericRespPayload(**raw_payload)
            else:
                self.logger.warning(f"[CODEC] Tipo de mensaje desconocido inyectado: {msg_type}")
                return self._build_internal_error_msg(StatusCode.BAD_REQUEST, f"Unknown message type: {msg_type}")
                
        except TypeError as e:
            self.logger.error(f"[CODEC] Faltan campos en el payload o su tipado de clases es incorrecto: {e}")
            return self._build_internal_error_msg(StatusCode.BAD_REQUEST, "Payload malformado en estructura Python.")

        return RobotMessage(header=header, payload=payload)

    def _build_internal_error_msg(self, code: int, description: str) -> RobotMessage:
        """!
        @brief Genera un paquete de error defensivo.
        @details Se utiliza cuando un mensaje crudo es tan defectuoso que no puede ni siquiera
                 instanciarse. Permite que el Director lo devuelva limpiamente al cliente.
        @param code Código HTTP-like de error (ej. 400 Bad Request).
        @param description Explicación detallada del fallo del parseo.
        @return Objeto RobotMessage de tipo PROTOCOL_ERROR.
        """
        header = MessageHeader(
            msg_id=-1, # Se asignará uno real y autoincremental al entrar en el Encoder
            type=MsgType.PROTOCOL_ERROR,
            session_id=""
        )
        payload = ProtocolErrorPayload(error_code=code, description=description)
        return RobotMessage(header=header, payload=payload)

    # =========================================================================
    # ENCODER (De Objetos Python a String JSON)
    # =========================================================================

    def encode(self, message: RobotMessage) -> str:
        """!
        @brief Transforma el modelo en memoria a texto de red (json).
        @details Inyecta metadatos del sistema (marcas de tiempo de salida, msg_id de respuestas)
                 y convierte las Dataclasses a un JSON optimizado.
        @param message Instancia completa de RobotMessage a enviar.
        @return Cadena String de JSON puro.
        """
        # 1. Autocompletar datos del sistema en la cabecera
        # Si el msg_id es <= 0 (Iniciativa del Servidor o Errores de Aduana), generamos uno nuevo.
        # Si ya tiene un ID válido (>0), lo respetamos (Eco de Respuestas Síncronas).
        if message.header.msg_id <= 0:
            message.header.msg_id = self._get_next_msg_id()
            
        message.header.timestamp = time.time()

        # 2. Convertir el objeto dataclass completo a diccionario nativo
        raw_dict = dataclasses.asdict(message)

        # 3. Limpieza y compresión de la trama
        clean_dict = self._remove_none_values(raw_dict)

        # 4. Empaquetado final a String
        return json.dumps(clean_dict)

    def _remove_none_values(self, data: Any) -> Any:
        """!
        @brief Purgador recursivo de claves vacías.
        @details Recorre el diccionario o listas en profundidad y elimina cualquier clave
                 cuyo valor sea `None`. Garantiza que el JSON emitido sea lo más pequeño posible
                 y evita incumplir restricciones del JSON Schema (como campos que esperan String o Nulo).
        @param data Estructura de datos (dict o list) a limpiar.
        @return La misma estructura libre de variables None.
        """
        if isinstance(data, dict):
            return {k: self._remove_none_values(v) for k, v in data.items() if v is not None}
        elif isinstance(data, list):
            return [self._remove_none_values(v) for v in data if v is not None]
        return data