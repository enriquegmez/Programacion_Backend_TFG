"""
main.py
El punto de entrada de la aplicación backend.
Inicializa la lógica, la red y la conexión con ROS 2.
"""

import asyncio
import logging
import sys

# Importamos todos nuestros módulos
from app_tiago.core.state_machine import ProtocolStateMachine
from app_tiago.communication.connection_manager import ConnectionManager
from app_tiago.core.router import MessageRouter
from app_tiago.ros.ros2_node_gateway import Ros2Manager
from app_tiago.communication.server import AppServer

def setup_logging():
    """Configura el formato de los logs para que se vean bonitos en la consola."""
    logging.basicConfig(
        level=logging.INFO, # Cambia a DEBUG si quieres ver los JSON crudos y los pings
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%H:%M:%S',
        handlers=[logging.StreamHandler(sys.stdout)]
    )

async def main():
    setup_logging()
    logger = logging.getLogger("Main")
    logger.info("=== INICIANDO BACKEND DE TELEOPERACIÓN ===")

    # 1. Instanciar el gestor de ROS 2 (El "Músculo")
    ros_manager = Ros2Manager()
    ros_manager.start() # Esto arranca rclpy en un hilo separado

    try:
        # 2. Instanciar la Máquina de Estados (El "Semáforo")
        state_machine = ProtocolStateMachine()

        # 3. Instanciar el Gestor de Conexiones (La "Seguridad")
        connection_manager = ConnectionManager(state_machine)

        # 4. Instanciar el Router inyectando dependencias (El "Cerebro")
        router = MessageRouter(connection_manager, state_machine, ros_node=ros_manager)

        # 5. Instanciar y arrancar el Servidor WebSocket (El "Cartero")
        server = AppServer(connection_manager, router)
        
        # El programa se quedará bloqueado aquí escuchando conexiones
        await server.start_server()

    except KeyboardInterrupt:
        logger.info("Interrupción por teclado detectada. Apagando servidor...")
    except Exception as e:
        logger.critical(f"Error fatal en la aplicación: {e}")
    finally:
        # Limpieza absoluta al apagar
        logger.info("Deteniendo subsistema ROS 2 y limpiando recursos...")
        ros_manager.stop()
        logger.info("=== BACKEND APAGADO ===")

def entry_point():
    """Envoltorio síncrono que ROS 2 puede ejecutar sin problemas."""
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    entry_point()