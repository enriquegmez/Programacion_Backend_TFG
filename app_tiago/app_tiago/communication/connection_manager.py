"""
connection_manager.py
Gestor de la conexión de red y la sesión.
Implementa el Watchdog (temporizador del Ping) y garantiza que solo 
haya un usuario controlando el robot a la vez.
"""

import asyncio
import logging
import time
import uuid

from app_tiago.utils.constants import ServerState
from app_tiago.core.state_machine import ProtocolStateMachine

# IMPORTANTE: Añade Any a tus imports para decirle a Mypy que el websocket 
# puede ser de cualquier tipo (ya que depende de la librería externa).
from typing import Any

# Importamos tu Ros2Manager para el type hinting
from app_tiago.ros.ros2_node_gateway import Ros2Manager

class ConnectionManager:
    # 🚨 ERROR 1 ARREGLADO: Inyectamos el ros2_manager en el constructor
    def __init__(self, state_machine: ProtocolStateMachine, ros2_manager: Ros2Manager):
        self.logger = logging.getLogger("ConnectionManager")
        self.state_machine = state_machine
        self.ros2_manager = ros2_manager # Guardamos la referencia
        
        self.active_websocket: Any | None = None
        self.current_session_id: str | None = None
        
        self.ping_timeout: float = 3.0 
        self.last_ping_time: float = 0.0
        self.watchdog_task: asyncio.Task[Any] | None = None

    # ==========================================
    # GESTIÓN FÍSICA (WebSockets)
    # ==========================================
    async def register_client(self, websocket) -> bool:
        """
        Llamado por server.py cuando un nuevo cliente intenta conectarse.
        Garantiza que solo haya 1 aplicación conectada al backend.
        """
        if self.active_websocket is not None:
            self.logger.warning(f"Conexión rechazada: El robot ya está siendo controlado por {self.active_websocket.remote_address[0]}")
            # Rechazamos la conexión con un código estándar de WebSocket (1008 Policy Violation)
            await websocket.close(code=1008, reason="El robot ya está en uso")
            return False

        self.logger.info("Cliente registrado como conexión principal.")
        self.active_websocket = websocket
        self.current_session_id = None
        
        # Iniciamos el temporizador de seguridad en cuanto se abre el puerto
        self.last_ping_time = time.time()
        self.watchdog_task = asyncio.create_task(self._watchdog_loop())
        return True

    async def unregister_client(self, websocket):
        """
        Llamado por server.py cuando el cliente se desconecta (voluntariamente o por error).
        """
        if self.active_websocket == websocket:
            self.logger.info("Liberando la conexión activa y limpiando sesión...")
            self.active_websocket = None
            self.current_session_id = None
            
            # Detenemos el vigilante del ping
            if self.watchdog_task:
                self.watchdog_task.cancel()
                self.watchdog_task = None
            
            # Avisamos a la máquina de estados para que pare el robot y vuelva al estado inicial
            self.state_machine.trigger_session_reset()

            # 🚨 ERROR 1 ARREGLADO: SEGURIDAD FÍSICA 🚨
            # Freno de emergencia incondicional en caso de desconexión.
            self.logger.warning("Parada de hombre muerto (Deadman Switch) activada: Frenando robot y cerrando conexión ROS 2.")
            self.ros2_manager.disconnect_from_robot()

    # ==========================================
    # GESTIÓN DE SESIÓN LÓGICA (Protocolo)
    # ==========================================
    def create_session(self) -> str:
        """Genera un ID único para la sesión cuando el cliente manda el comando 'connect'."""
        # uuid4 genera un identificador único aleatorio tipo "f47ac10b-58cc-4372-a567-0e02b2c3d479"
        self.current_session_id = str(uuid.uuid4())
        self.logger.info(f"Nueva sesión lógica generada: {self.current_session_id}")
        return self.current_session_id

    def is_valid_session(self, session_id: str) -> bool:
        """Comprueba si el mensaje entrante pertenece a la sesión activa."""
        if self.state_machine.global_state == ServerState.CONEXION_BACKEND:
            # Antes de conectarse, el frontend no tiene un ID válido, así que lo permitimos
            return True
            
        # Si la sesión ya está iniciada, el ID debe coincidir exactamente
        return self.current_session_id == session_id

    # ==========================================
    # WATCHDOG (Seguridad y Heartbeat)
    # ==========================================
    def notify_ping_received(self):
        """
        El router llamará a este método cada vez que reciba un PING_REQ del móvil.
        Esto reinicia el contador de tiempo.
        """
        self.last_ping_time = time.time()

    async def _watchdog_loop(self):
        """
        Bucle asíncrono continuo. Desconecta al usuario si el ping no llega a tiempo.
        """
        try:
            while True:
                # Revisamos el temporizador cada segundo
                await asyncio.sleep(1.0)
                elapsed_time = time.time() - self.last_ping_time
                
                if elapsed_time > self.ping_timeout:
                    self.logger.error(f"¡TIMEOUT CRÍTICO! Han pasado {elapsed_time:.1f}s sin Heartbeat (PING).")
                    
                    if self.active_websocket:
                        # Si cortamos el websocket, se lanzará una excepción en server.py
                        # que a su vez llamará a nuestro unregister_client en su bloque 'finally'
                        await self.active_websocket.close(code=1000, reason="Ping Timeout - Conexión perdida")
                    break
                    
        except asyncio.CancelledError:
            # Esto ocurre limpiamente cuando llamamos a watchdog_task.cancel()
            self.logger.debug("Watchdog detenido correctamente.")