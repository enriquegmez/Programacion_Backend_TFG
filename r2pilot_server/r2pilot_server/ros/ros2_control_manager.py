## @file ros2_control_manager.py
#  @brief Gestor de Cinemática y Teleoperación en tiempo real.
#  @details Aísla la lógica de publicación de comandos de velocidad (Twist) y 
#           posicionamiento de articulaciones (JointTrajectory), gestionando
#           el semáforo lógico de control activo.
#  @author Enrique Gómez
#  @date 2026

import logging
from typing import Any, Dict, Tuple

from geometry_msgs.msg import Twist                                   # type: ignore[import]
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # type: ignore[import]

from r2pilot_server.utils.constants import ControlEvent

class ControlManager:
    """!
    @brief Administrador de los actuadores físicos del robot.
    @details Se encarga de inyectar las velocidades en el cortafuegos de seguridad
             (SafetyFilter) y de enrutar las peticiones angulares a los controladores
             de hardware dinámicos descubiertos por el URDF.
    """
    def __init__(self, node: Any) -> None:
        """!
        @brief Inicializa los publicadores y el estado del semáforo.
        @param node Referencia al nodo principal de ROS 2.
        """
        self.node = node
        self.logger = logging.getLogger("ControlManager")
        
        # Publicador fijo hacia el tópico RAW que lee el R2PilotSafetyNode
        self.vel_publisher = self.node.create_publisher(Twist, 'R2Pilot_teleop/cmd_vel_raw', 10)
        self.joint_publishers: Dict[str, Any] = {}
        
        self.is_control_active = False

    def set_control_mode(self, event: str, control_type: str = "TELEOP", topic: str = "cmd_vel") -> Tuple[bool, str]:
        """!
        @brief Abre o cierra el semáforo lógico para permitir inyección de velocidades.
        @details Controla el estado de la bandera `is_control_active` y valida que el tópico
                 de destino exista y sea compatible con Twist o JointTrajectory según corresponda.
        @param event Tipo de evento: START o STOP.
        @param control_type Tipo de control: TELEOP o JOINT.
        @param topic Nombre del tópico de destino para TELEOP (por defecto "cmd_vel").
        @return Tupla (éxito, mensaje). Éxito es True si se pudo cambiar el estado, y el mensaje contiene información adicional 
                o error.
        """
        if not self.node.is_connected: 
            return False, "No conectado lógicamente."
        
        # Validamos el evento y el tipo de control
        if event == ControlEvent.START:
            self.is_control_active = True
            if control_type != "JOINT":
                clean = topic.strip()
                for name, types in self.node.get_topic_names_and_types():
                    if name == clean or name.endswith(f'/{clean.lstrip("/")}') or name == f'/{clean}':
                        if 'geometry_msgs/msg/Twist' in types: 
                            return True, name
                        return False, "Topic no acepta Twist."
                self.is_control_active = False
                return False, "Topic no encontrado."
            return True, "JOINT_CONTROL"
            
        elif event == ControlEvent.STOP:
            self.is_control_active = False
            if control_type != "JOINT": 
                self.stop_robot()
            return True, ""
            
        return False, "Evento desconocido."

    def publish_velocity(self, v: float, w: float) -> Tuple[bool, str]: 
        """!
        @brief Puentea la intención de movimiento hacia el canal seguro.
        @details Escucha al SafetyFilter para rechazar movimientos que puedan acabar en colisión.
        @param v Velocidad lineal deseada (m/s).
        @param w Velocidad angular deseada (rad/s).
        @return Tupla (éxito, mensaje). Éxito es True si se pudo publicar el comando, 
                y el mensaje contiene información adicional o error.
        """
        if not self.node.is_connected or not self.is_control_active: 
            return False, "Control no activo."
            
        # Leemos la alerta almacenada en el nodo principal
        if v > 0.0 and self.node.safety_alert in ["FRONT", "BOTH"]: 
            return False, "ANTICOLISIÓN FRONT."
        if v < 0.0 and self.node.safety_alert in ["REAR", "BOTH"]: 
            return False, "ANTICOLISIÓN REAR."

        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.vel_publisher.publish(msg)
        return True, ""

    def stop_robot(self) -> None:
        """! 
        @brief Publica un mensaje Twist con todo a ceros al canal seguro. 
        @details Se utiliza para detener el robot de forma inmediata al cerrar el semáforo lógico.
        """
        try:
            msg = Twist()
            msg.linear.x, msg.angular.z = 0.0, 0.0
            self.vel_publisher.publish(msg)
        except Exception as e:
            self.logger.debug(f"Freno omitido: {e}")

    def publish_joint_position(self, joint_name: str, value: float) -> bool:
        """! 
        @brief Publica un punto de trayectoria simple hacia los controladores URDF.
        @details Comprueba restricciones mecánicas del DiscoveryManager para evitar comandos fuera de rango.
        @param joint_name Nombre de la articulación a mover.
        @param value Valor angular deseado (radianes).
        @return True si se pudo publicar el comando, False si hubo error o el control no estaba activo.
        """
        if not self.node.is_connected or not self.is_control_active: 
            return False

        # 1. Recortamos a los límites de hardware descubiertos
        if self.node.discovery.joint_limits:
            for limit in self.node.discovery.joint_limits:
                if limit["name"] == joint_name:
                    value = max(limit["min"], min(limit["max"], value))
                    break

        # 2. Buscamos a qué topic hay que mandarlo
        target_topic, expected_joints = None, []
        for topic, joints in self.node.discovery.dynamic_controllers.items():
            if joint_name in joints:
                target_topic, expected_joints = topic, joints
                break
                
        if not target_topic: 
            return True # Ignoramos temporalmente sin desconectar el WebSocket

        if target_topic not in self.joint_publishers:
            print(target_topic)
            self.joint_publishers[target_topic] = self.node.create_publisher(JointTrajectory, target_topic, 10)

        # 3. Construimos la trama de ROS 2
        msg = JointTrajectory()
        msg.joint_names = expected_joints
        
        point = JointTrajectoryPoint()
        # Inyectamos el valor nuevo para la articulación deseada, y mantenemos el valor actual para el resto
        point.positions = [float(value) if j == joint_name else float(self.node.current_joint_states.get(j, 0.0)) for j in expected_joints]
        point.time_from_start.nanosec = 200000000 # 200ms para suavizar el movimiento
        
        msg.points = [point]
        self.joint_publishers[target_topic].publish(msg)
        return True