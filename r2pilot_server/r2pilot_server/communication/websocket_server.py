## @file websocket_server.py
#  @brief El Servidor asíncrono de la comunicación con el cliente móvil basado en WebSockets.
#  @details Se encarga exclusivamente de levantar el puerto de escucha, mantener el bucle 
#           asíncrono de recepción, enviar los mensajes por la red mediante WebSockets
#           y derivar el procesamiento lógico de los mensajes (deserialización) al Director central.
#  @author Enrique Gómez
#  @date 2026

import logging
import time # [TFG] Importación necesaria para el reloj de alta precisión

from websockets.asyncio.server import serve, ServerConnection
from websockets.exceptions import ConnectionClosed

from r2pilot_server.utils.constants import SERVER_IP, SERVER_PORT
from r2pilot_server.communication.connection_manager import ConnectionManager
from r2pilot_server.core.director import Director


class WebsocketServer:
    """!
    @brief Servidor asíncrono basado en WebSockets.
    @details Gestiona el ciclo de vida de los sockets físicos (Apertura, Recepción en bucle, Cierre)
             y actúa como puente inyector de dependencias (Callbacks de envío) para que las 
             capas lógicas (Router) no dependan de librerías de red específicas.
    """

    def __init__(self, connection_manager: ConnectionManager, director: Director) -> None:
        """!
        @brief Inicializa el servidor inyectando sus dependencias lógicas.
        @param connection_manager Gestor de exclusión mutua y seguridad del enlace.
        @param director Director central que procesará los mensajes entrantes.
        """
        self.logger = logging.getLogger("WebsocketServer")
        self.connection_manager = connection_manager
        self.director = director

    async def send_message(self, websocket: ServerConnection, message: str) -> None:
        """!
        @brief Punto único de emisión de red (Salida en embudo).
        @details Cualquier otro script de la aplicación que desee enviar datos al cliente 
                 debe pasar obligatoriamente por esta función. Incluye captura segura de excepciones.
        @param websocket El socket físico abierto con el cliente móvil.
        @param message El mensaje previamente serializado que se enviará.
        """
        try:
            await websocket.send(message)
        except ConnectionClosed:
            self.logger.warning("[RED] Intento de envío abortado: El cliente ya se había desconectado físicamente.")
        except Exception as e:
            self.logger.error(f"[RED] Fallo crítico al enviar datos por WebSocket: {e}")

    async def handler(self, websocket: ServerConnection) -> None:
        """!
        @brief Bucle principal de atención al cliente (Lanzado por cada nueva conexión).
        @details Delega la validación de acceso al `ConnectionManager`, genera los callbacks 
                 de inyección y procesa en bucle infinito los paquetes entrantes hasta que el 
                 socket se destruye.
        @param websocket El socket físico instanciado por la librería `websockets`.
        """
        client_ip = websocket.remote_address[0] if hasattr(websocket, "remote_address") else "IP Desconocida"
        self.logger.info(f"[RED] Nueva petición de conexión entrante desde la IP: {client_ip}")

        # 1. Filtro de exclusión mutua: Verificamos si el robot está libre
        is_accepted = await self.connection_manager.register_client(websocket)
        if not is_accepted:
            # Si el robot está ocupado, register_client ya ha cerrado el socket internamente.
            return

        # 2. Generamos el token de sesión lógico
        session_id = self.connection_manager.create_session()

        # =====================================================================
        # FÁBRICA DE CALLBACKS (Inyección de Dependencias para el Director)
        # Así independizamos estas funciones del uso de Websockets por si se requiere otro tipo de comunicación en el futuro 
        # =====================================================================
        
        async def send_callback(message: str) -> None:
            """!
            @brief Callback inyectable de envío de datos de red.
            @details Permite a las capas de lógica superior (como el Router) emitir mensajes de vuelta
                     al dispositivo móvil sin tener dependencia directa con el socket de red físico.
            @param message El mensaje previamente serializado que se enviará.
            """
            await self.send_message(websocket, message)

        async def close_callback() -> None:
            """@brief Callback inyectable de finalización y cierre físico de la conexión.
            @details Permite a las capas superiores de control lógico forzar la finalización limpia 
                     del canal físico enviando un código de cierre normal (1000 - RFC 6455). 
            """
            self.logger.info("[RED] El Director ha solicitado el cierre programado de la conexión física.")
            await websocket.close(code=1000, reason="Cierre limpio solicitado por protocolo (END)")

        # 3. Primer paquete de la conexión: Enviamos el ID de sesión asignado al cliente
        await self.director.send_session_assigned(session_id, send_callback)
        
        try:
            # 4. BUCLE DE ESCUCHA INFINITO
            async for raw_message in websocket:
                
                # Aseguramos el tipado a string puro para evitar errores de codificación
                if isinstance(raw_message, bytes):
                    text_message = raw_message.decode("utf-8")
                else:
                    text_message = raw_message

                # Ocultamos el log crudo si es un paquete muy grande para no saturar la terminal
                if "sensor_msgs" not in text_message:
                    self.logger.debug(f"[RED IN] Mensaje crudo recibido: {text_message}")
                
                # Derivamos la trama de texto pura a la capa lógica
                await self.director.handle_raw_message(text_message, send_callback, close_callback)

        except ConnectionClosed as e:
            # [TFG] T1 FÍSICO: La red ha sido cortada físicamente (Cierre esperado/Limpiado por SO)
            self.connection_manager.t1_emergencia = time.perf_counter_ns()
            self.logger.warning(f"[RED] Cliente desconectado (Cierre físico detectado): {e}")
        except Exception as e:
            # [TFG] T1 FÍSICO: Fallo crítico repentino de hardware/red
            self.connection_manager.t1_emergencia = time.perf_counter_ns()
            self.logger.error(f"[RED] Error crítico no controlado en la conexión con {client_ip}: {e}")
        finally:
            # 5. GARANTÍA DE LIMPIEZA (Se ejecuta SIEMPRE, haya fallado o no)
            await self.connection_manager.unregister_client(websocket)
            self.logger.info(f"[RED] Conexión física con {client_ip} destruida y memoria liberada.")

    async def start_server(self) -> None:
        """!
        @brief Levanta el socket del servidor asíncrono y bloquea el hilo escuchando peticiones.
        @details Utiliza la IP y el Puerto configurados globalmente en la clase de constantes.
        @return None
        """
        self.logger.info(f"[SISTEMA] Arrancando servidor maestro de teleoperación en ws://{SERVER_IP}:{SERVER_PORT}")
        # 'serve' maneja de forma automática la creación de tareas asíncronas por cada cliente que llama al 'handler'
        async with serve(self.handler, SERVER_IP, SERVER_PORT) as server:
            await server.serve_forever()