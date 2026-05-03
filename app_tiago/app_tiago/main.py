"""
main.py
Punto de entrada principal del backend de teleoperación del TIAGO.
"""

import asyncio
import logging

# Importamos nuestro servido
from app_tiago.communication.server import AppServer

def setup_logging():
    """Configura el formato de los mensajes que veremos en la terminal."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S"
    )

def main():
    setup_logging()
    logger = logging.getLogger("Main")
    logger.info("Arrancando sistema base del TIAGO...")

    # Instanciamos la clase del servidor
    server = AppServer()

    try:
        # asyncio.run() es la forma oficial de ejecutar una función asíncrona desde código normal
        asyncio.run(server.start_server())
    except KeyboardInterrupt:
        # Esto captura cuando pulsas Ctrl+C en la terminal para apagarlo limpiamente
        logger.info("Apagado manual detectado (Ctrl+C). Cerrando servidor.")
    except Exception as e:
        logger.error(f"Error crítico en el servidor: {e}")

if __name__ == "__main__":
    main()