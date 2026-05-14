"""
server.py
El "Cartero" de la aplicación.
Se encarga exclusivamente de levantar el puerto, mantener el bucle de recepción,
enviar los mensajes por la red y derivar el procesamiento lógico al Router.
"""

import logging
from websockets.asyncio.server import serve, ServerConnection
from websockets.exceptions import ConnectionClosed

from app_tiago.utils.constants import SERVER_IP, SERVER_PORT

class AppServer:
    def __init__(self, connection_manager, router):
        self.logger = logging.getLogger("AppServer")
        self.connection_manager = connection_manager
        self.router = router

    async def send_message(self, websocket: ServerConnection, json_string: str):
        """
        ÚNICO punto de toda la aplicación donde se envía información por la red.
        Cualquier otro script que quiera enviar datos debe pasar por aquí.
        """
        try:
            await websocket.send(json_string)
        except ConnectionClosed:
            self.logger.warning("Intento de envío fallido: El cliente ya se había desconectado.")
        except Exception as e:
            self.logger.error(f"Fallo crítico al enviar datos por WebSocket: {e}")

    async def handler(self, websocket: ServerConnection):
        client_ip = websocket.remote_address[0]
        self.logger.info(f"Nueva conexión entrante desde la IP: {client_ip}")

        await self.connection_manager.register_client(websocket)
        #Creamos el session id
        session_id = self.connection_manager.create_session()
        # Creamos la función que inyectaremos en el router.
        # Así el router puede enviar mensajes sin saber qué es un "websocket".
        async def send_callback(json_str: str):
            await self.send_message(websocket, json_str)

        async def close_callback():
            self.logger.info("El Router ha solicitado el cierre de la conexión física.")
            await websocket.close(code=1000, reason="Cierre limpio por END")

        #enviamos el session id al cliente
        await self.router.send_session_assigned(session_id, send_callback)
        
        try:
            async for raw_message in websocket:
                # 1. Aseguramos que el mensaje es puro texto (str)
                # Si llega como binario (bytes), lo decodificamos a UTF-8.
                if isinstance(raw_message, bytes):
                    text_message = raw_message.decode("utf-8")
                else:
                    text_message = raw_message

                # 2. Ahora ya podemos imprimirlo sin que Mypy se enfade
                self.logger.debug(f"Mensaje crudo recibido: {text_message}")
                
                # Le pasamos el string crudo y la "tubería" de salida
                await self.router.handle_raw_message(raw_message, send_callback, close_callback)

        except ConnectionClosed as e:
            self.logger.warning(f"Cliente desconectado de forma esperada/inesperada: {e}")
        except Exception as e:
            self.logger.error(f"Error crítico en la conexión con {client_ip}: {e}")
        finally:
            await self.connection_manager.unregister_client(websocket)
            self.logger.info(f"Conexión con {client_ip} cerrada y limpiada.")

    async def start_server(self):
        self.logger.info(f"Iniciando servidor de teleoperación en ws://{SERVER_IP}:{SERVER_PORT}")
        async with serve(self.handler, SERVER_IP, SERVER_PORT) as server:
            await server.serve_forever()