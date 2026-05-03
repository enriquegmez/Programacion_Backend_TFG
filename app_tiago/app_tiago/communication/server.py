"""
server_node.py
El servidor WebSocket asíncrono. Actúa como la puerta de enlace física entre 
la red y la lógica interna de Python.
"""

import asyncio
import logging

# ¡Usando la API Moderna de WebSockets!
from websockets.asyncio.server import serve, ServerConnection
from websockets.exceptions import ConnectionClosed

# Importamos nuestras herramientas
from app_tiago.utils.constants import SERVER_IP, SERVER_PORT, MsgType, Action
from app_tiago.protocol.json_translator import MessageCodec

class AppServer:
    def __init__(self):
        self.logger = logging.getLogger("AppServer")
        self.codec = MessageCodec()
        self.active_connections = set()

    # Añadimos el tipo ServerConnection para tener autocompletado de los métodos send() y recv()
    async def handler(self, websocket: ServerConnection):
        """
        Esta función se ejecuta CADA VEZ que un nuevo móvil se conecta.
        """
        client_ip = websocket.remote_address[0]
        self.logger.info(f"Nuevo cliente conectado desde la IP: {client_ip}")
        self.active_connections.add(websocket)

        try:
            # Bucle infinito escuchando los mensajes
            async for raw_message in websocket:
                self.logger.info(f"Mensaje crudo recibido: {raw_message}")

                # 1. PARSER
                mensaje_dict = self.codec.decode(raw_message)

                # 2. ROUTER IMPROVISADO (Solo para la prueba)
                header = mensaje_dict.get("header", {})
                payload = mensaje_dict.get("payload", {})

                if header.get("type") == MsgType.COMMAND_REQ and payload.get("action") == Action.CONNECT:
                    self.logger.info("Recibida petición de conexión. Procesando...")
                    
                    cliente_session_id = header.get("session_id", "temp_id")
                    assigned_id_int = 1 
                    
                    # 3. FACTORY
                    json_respuesta = self.codec.build_connect_success(cliente_session_id, assigned_id_int)
                    
                    # 4. ENVÍO
                    await websocket.send(json_respuesta)
                    self.logger.info("Respuesta de éxito enviada al cliente.")
                
                elif header.get("type") == MsgType.PROTOCOL_ERROR:
                    self.logger.warning("El JSON estaba mal formado. Enviando error de protocolo.")
                    error_json = self.codec.build_protocol_error(400, "Invalid JSON format")
                    await websocket.send(error_json)

                else:
                    self.logger.warning(f"Mensaje ignorado. Tipo recibido: {header.get('type')}")

        except ConnectionClosed as e:
            self.logger.warning(f"Cliente desconectado: {e}")
        finally:
            self.active_connections.remove(websocket)
            self.logger.info(f"Conexión con {client_ip} cerrada y limpiada.")

    async def start_server(self):
        """Arranca el servidor usando la API moderna de websockets"""
        self.logger.info(f"Iniciando servidor de teleoperación en ws://{SERVER_IP}:{SERVER_PORT}")
        
        # Uso del gestor de contexto asíncrono con serve_forever()
        async with serve(self.handler, SERVER_IP, SERVER_PORT) as server:
            await server.serve_forever()