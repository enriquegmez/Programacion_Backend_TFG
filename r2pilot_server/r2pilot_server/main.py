## @file main.py
#  @brief Función Principal del servidor R2Pilot.
#  @details Orquesta la inicialización de toda la arquitectura mediante inyección 
#           de dependencias, uniendo el motor de ROS 2, la máquina de estados lógicos 
#           y el servidor asíncrono de WebSockets. Garantiza un apagado seguro.
#  @author Enrique Gómez
#  @date 2026

import asyncio
import logging
import sys

# Importación de los Módulos de la Arquitectura
from r2pilot_server.core.state_machine import ProtocolStateMachine
from r2pilot_server.communication.connection_manager import ConnectionManager
from r2pilot_server.core.director import Director
from r2pilot_server.ros.ros2_node_gateway import Ros2Manager
from r2pilot_server.communication.websocket_server import WebsocketServer  

def setup_logging() -> None:
    """!
    @brief Configura el formato estándar de la consola.
    @details Ajusta el formato para incluir timestamps precisos y el nivel de severidad,
             facilitando la depuración visual del tráfico de red y eventos de ROS 2.
    """
    logging.basicConfig(
        level=logging.INFO, # Cambiar a DEBUG para trazar los JSON crudos y los PING_REQ
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%H:%M:%S',
        handlers=[logging.StreamHandler(sys.stdout)]
    )

async def main() -> None:
    """!
    @brief Bucle principal de ejecución asíncrona.
    @details Sigue el patrón de "Inyección de Dependencias". Instancia los componentes 
             desde los más abstractos a los más concretos, y bloquea el hilo principal 
             dejando el servidor web a la escucha.
    """
    setup_logging()
    logger = logging.getLogger("Main")
    logger.info("=== INICIANDO BACKEND DE TELEOPERACIÓN R2PILOT ===")

    # 1. Instanciar la librería de ROS 2 y arrancar el SafetyFilter en un hilo separado
    ros_manager = Ros2Manager()
    ros_manager.start() # Arranca rclpy y el SafetyFilter en un hilo del SO separado

    try:
        # 2. Instanciar la Máquina de Estados Lógica (Dominio / Semáforo)
        state_machine = ProtocolStateMachine()

        # 3. Instanciar el Gestor de Sesiones (Seguridad / Controlador de Acceso)
        connection_manager = ConnectionManager(state_machine, ros2_manager=ros_manager)

        # 4. Instanciar el Director (Enrutador Semántico / Cerebro)
        director = Director(connection_manager, state_machine, ros_node=ros_manager)

        # 5. Instanciar y arrancar el Servidor WebSocket (Transporte / Cartero)
        server = WebsocketServer(connection_manager, director)
        
        # Bloqueo asíncrono: El programa se queda aquí procesando los paquetes entrantes
        await server.start_server()

    except asyncio.CancelledError:
        logger.info("Tarea asíncrona cancelada. Apagando servidor...")
    except Exception as e:
        logger.critical(f"Error fatal no controlado en la aplicación: {e}")
    finally:
        # Apagado Controlado (Graceful Shutdown)
        # Garantiza que el robot aplique el freno de emergencia y libere la memoria RAM
        logger.info("Deteniendo subsistema ROS 2 y limpiando recursos físicos...")
        ros_manager.stop()
        logger.info("=== BACKEND APAGADO ===")

def entry_point() -> None:
    """!
    @brief Envoltorio síncrono para la ejecución desde un entrypoint de ROS 2 (setup.py).
    @details Captura interrupciones de teclado (Ctrl+C) en la terminal para permitir el apagado limpio.
    """
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        # Se captura el SIGINT (Ctrl+C) en silencio, ya que main() gestiona la limpieza en su finally
        pass

if __name__ == "__main__":
    entry_point()