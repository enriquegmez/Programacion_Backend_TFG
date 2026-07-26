## @file ros2_action_manager.py
#  @brief Controlador de acciones complejas (PlayMotion2) en ROS 2.
#  @details Contiene los clientes de acción, el gestor de promesas asíncronas (Futures) 
#           y el simulador de progreso para el frontend.
#  @author Enrique Gómez
#  @date 2026

import time
import logging
import threading
from typing import Any, Callable, Optional, Tuple

import rclpy                                          # type: ignore[import]
from rclpy.action import ActionClient, CancelResponse # type: ignore[import]
from rclpy.timer import Timer                         # type: ignore[import]
from rclpy.client import Client                       # type: ignore[import]

from play_motion2_msgs.action import PlayMotion2      # type: ignore[import]
from play_motion2_msgs.srv import GetMotionInfo, ListMotions # type: ignore[import]

class ActionManager:
    """!
    @brief Gestor del ciclo de vida de las acciones de PlayMotion2.
    @details Se encarga de enviar órdenes, recibir feedback, cancelar acciones y simular
             el progreso para el frontend.
    """
    def __init__(self, node: Any) -> None:
        self.node = node
        self.logger = logging.getLogger("ActionManager")
        
        self.play_motion_initialized = False
        self.play_motion_action_client: Optional[ActionClient] = None
        self.list_motions_client: Optional[Client] = None
        self.get_motion_info_client: Optional[Client] = None
        
        self.current_goal_handle: Optional[Any] = None
        self.current_action_name: Optional[str] = None
        self._action_lock = threading.Lock()
        
        self._router_feedback_callback: Optional[Callable[[bool, bool, int, str], None]] = None

        self.action_progress_timer: Optional[Timer] = None
        self.action_start_time: float = 0.0
        self.current_motion_total_duration: float = 0.0
        self._fake_progress: int = 0
        self.was_canceled_by_user: bool = False

    def _wait_for_service(self, client: Any, timeout_sec: float = 5.0) -> bool:
        """!
        @brief Espera a que un servicio esté disponible.
        @param client Cliente de servicio de ROS 2.
        @param timeout_sec Tiempo máximo de espera en segundos.
        @return True si el servicio está listo, False si se agotó el tiempo.
        """
        if client.service_is_ready(): return True
        return client.wait_for_service(timeout_sec=timeout_sec)

    def _wait_for_future(self, future: Any, timeout_sec: float = 5.0) -> bool:
        """!
        @brief Espera a que un Future de ROS 2 se complete.
        @param future Future de ROS 2.
        @param timeout_sec Tiempo máximo de espera en segundos.
        @return True si el Future se completó, False si se agotó el tiempo.
        """
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done()

    def discover_endpoints(self) -> bool:
        """! 
        @brief Escanea la red en busca de los servicios de PlayMotion2. 
        @return True si se encontraron los servicios, False en caso contrario.
        """
        if self.play_motion_initialized: return True
        services = self.node.get_service_names_and_types()
        candidates = [name for name, _ in services if 'play_motion' in name and name.endswith('/list_motions')]

        if not candidates: return False

        base = candidates[0].rsplit('/', 1)[0]
        self.play_motion_action_client = ActionClient(self.node, PlayMotion2, base)
        self.list_motions_client = self.node.create_client(ListMotions, candidates[0])
        self.get_motion_info_client = self.node.create_client(GetMotionInfo, f"{base}/get_motion_info")

        self.play_motion_initialized = True
        return True

    def get_available_actions(self) -> Tuple[bool, Any]:
        """! 
        @brief Obtiene la lista de movimientos soportados por el robot.
        @return Tupla (éxito, lista de movimientos o mensaje de error).
        """
        if not self.discover_endpoints() or self.list_motions_client is None:
            return False, "No se encontró PlayMotion."

        if not self._wait_for_service(self.list_motions_client, timeout_sec=3.0): 
            return False, "Servicio no disponible."

        future = self.list_motions_client.call_async(ListMotions.Request())
        if not self._wait_for_future(future, timeout_sec=4.0): return False, "Timeout consultando."

        result = future.result()
        if result is None or not result.motion_keys: return False, "La lista está vacía."
        return True, list(result.motion_keys)

    def execute_action(self, target: str) -> Tuple[bool, str]:
        """! 
        @brief Envía la orden física y levanta los monitores. 
        @param target Nombre del movimiento a ejecutar.
        @return Tupla (éxito, mensaje de estado).
        """
        if not self.discover_endpoints() or self.play_motion_action_client is None:
            return False, "Interfaz no disponible."

        # Consulta el tiempo total
        self.current_motion_total_duration = 0.0
        if self.get_motion_info_client and self._wait_for_service(self.get_motion_info_client, 2.0):
            req = GetMotionInfo.Request()
            req.motion_key = target
            f_info = self.get_motion_info_client.call_async(req)
            if self._wait_for_future(f_info, 2.0) and f_info.result() and f_info.result().motion:
                times = getattr(f_info.result().motion, 'times_from_start', [])
                if times: self.current_motion_total_duration = float(times[-1]) + 2.5 

        self._fake_progress = 0
        self.action_start_time = time.time()
        
        # Levantamos el temporizador de progreso (simulación para el frontend)
        if self.action_progress_timer: self.action_progress_timer.cancel()
        self.action_progress_timer = self.node.create_timer(0.25, self._progress_timer_callback)
        
        # Enviamos la orden de movimiento
        goal_msg = PlayMotion2.Goal()
        goal_msg.motion_name = target
        goal_msg.skip_planning = False

        goal_future = self.play_motion_action_client.send_goal_async(goal_msg)
        if not self._wait_for_future(goal_future, timeout_sec=5.0): return False, "Timeout enviando orden."

        goal_handle = goal_future.result()
        if not goal_handle.accepted: return False, "Robot rechazó el movimiento."

        with self._action_lock:
            self.current_goal_handle = goal_handle
            self.current_action_name = target

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._play_motion_result_callback)

        return True, "Ejecutando..."

    def stop_action(self, target: str) -> bool:
        """! 
        @brief Cancela un Goal activo en ROS 2. 
        @param target Nombre del movimiento a cancelar.
        @return True si se canceló correctamente, False en caso contrario.
        """
        with self._action_lock:
            handle = self.current_goal_handle
            if handle is None or target != self.current_action_name: return False
            
            self.was_canceled_by_user = True 
            cancel_future = handle.cancel_goal_async()

        if not self._wait_for_future(cancel_future, timeout_sec=5.0): return False
        if cancel_future.result().return_code == CancelResponse.ACCEPT:
            with self._action_lock:
                self.current_goal_handle = None
                self.current_action_name = None
            return True
        return False

    def set_feedback_callback(self, callback: Callable[[bool, bool, int, str], None]) -> None:
        """!
        @brief Registra un callback para enviar feedback al frontend.
        @param callback Función que recibe (éxito, finalizado, progreso, mensaje).
        """
        self._router_feedback_callback = callback

    def _progress_timer_callback(self) -> None:
        """!
        @brief Temporizador que simula el progreso de la acción para el frontend.
        @details Se ejecuta periódicamente mientras la acción está en curso y envía actualizaciones de progreso.
        """
        if not self.current_goal_handle:
            if self.action_progress_timer: self.action_progress_timer.cancel()
            return

        elapsed = time.time() - self.action_start_time
        if self.current_motion_total_duration > 0:
            progress = min(95, max(0, int((elapsed / self.current_motion_total_duration) * 100)))
        else:
            self._fake_progress += 5
            progress = min(95, self._fake_progress)

        if self._router_feedback_callback:
            self._router_feedback_callback(True, False, progress, f"Progreso: {progress}%")

    def _play_motion_result_callback(self, future: Any) -> None:
        """!
        @brief Callback que se ejecuta cuando la acción de PlayMotion2 finaliza.
        @param future Future que contiene el resultado de la acción.
        """
        if self.action_progress_timer: self.action_progress_timer.cancel()
        try:
            result = future.result().result
            if getattr(self, 'was_canceled_by_user', False):
                self.was_canceled_by_user = False 
                if self._router_feedback_callback: self._router_feedback_callback(True, True, 100, "Acción detenida por el usuario")
            elif result.success:
                if self._router_feedback_callback: self._router_feedback_callback(True, True, 100, "Acción completada con éxito")
            else:
                error_msg = str(getattr(result, 'error', 'Error desconocido'))
                if "cancel" in error_msg.lower():
                    if self._router_feedback_callback: self._router_feedback_callback(True, True, 100, "Acción detenida por el usuario")
                else:
                    if self._router_feedback_callback: self._router_feedback_callback(False, True, 0, f"Acción fallida: {error_msg}")
        except Exception as e:
            self.logger.error(f"Excepción en el resultado de la acción: {e}")
        finally:
            with self._action_lock:
                self.current_goal_handle = None
                self.current_action_name = None

    def cancel_all(self) -> None:
        """! 
        @brief Aborta forzosamente (usado en desconexión). 
        @details Cancela cualquier acción en curso y limpia los estados internos.
        """
        with self._action_lock:
            # MAGIA MYPY: Lo guardamos en una variable local (handle)
            handle = self.current_goal_handle
            if handle is not None:
                self.logger.info("Cancelando movimiento por desconexión...")
                try:
                    self.was_canceled_by_user = True 
                    handle.cancel_goal_async()
                except Exception as e:
                    self.logger.warning(f"Aviso al cancelar: {e}")
                finally:
                    self.current_goal_handle = None
                    self.current_action_name = None
                    
        if self.action_progress_timer: self.action_progress_timer.cancel()