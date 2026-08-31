## @file ros2_safety_node.py
#  @brief Nodo independiente de seguridad y anticolisión de ROS 2.
#  @details Actúa como un cortafuegos (proxy) entre los comandos de la app web y los
#           controladores de hardware del robot. Implementa descubrimiento dinámico de 
#           LiDARs, análisis de obstáculos y limitación de velocidades.
#  @author Enrique Gómez
#  @date 2026

import time
import math
import logging

import rclpy                                      # type: ignore[import]
from rclpy.node import Node                       # type: ignore[import]
from geometry_msgs.msg import Twist               # type: ignore[import]
from sensor_msgs.msg import LaserScan             # type: ignore[import]
from std_msgs.msg import String                   # type: ignore[import]
from rclpy.qos import qos_profile_sensor_data     # type: ignore[import]

from typing import Dict, Any, Optional

from r2pilot_server.utils.constants import RobotLimits


class R2PilotSafetyNode(Node):
    """!
    @brief Filtro de seguridad reactivo para teleoperación.
    @details Escucha los comandos crudos generados por el joystick del usuario, 
             evalúa el entorno del robot mediante los sensores láser suscritos dinámicamente, 
             y publica velocidades seguras al controlador final del robot.
    """

    def __init__(self) -> None:
        """!
        @brief Constructor del nodo de seguridad de ROS 2.
        @details Inicializa los suscriptores, las estructuras de memoria para los LiDARs
                 y los umbrales estáticos de seguridad matemática.
        """
        super().__init__('R2Pilot_safety_filter')
        
        self.py_logger = logging.getLogger("R2PilotSafetyNode")

        # 1. Entrada de control inexperto (Desde el servidor WebSocket)
        self.raw_sub = self.create_subscription(Twist, 'R2Pilot_teleop/cmd_vel_raw', self.vel_callback, 10)
        
        # 2. Canal de salida de notificaciones (Hacia la app móvil)
        self.alert_pub = self.create_publisher(String, 'R2Pilot_teleop/safety_alert', 10)
        
        self.target_topic: Optional[str] = None
        self.real_robot_pub: Optional[Any] = None
        
        # =====================================================================
        # UMBRALES ABSOLUTOS DE SEGURIDAD (Cinemática)
        # =====================================================================
        self.MAX_V = RobotLimits.MAX_LINEAR_VEL  # Velocidad lineal máxima permitida (m/s)
        self.MAX_W = RobotLimits.MAX_ANGULAR_VEL  # Velocidad angular máxima permitida (rad/s)
        
        # =====================================================================
        # VARIABLES DEL ESCUDO ANTICOLISIÓN (Geometría)
        # =====================================================================
        self.SAFE_DIST = RobotLimits.SAFE_DIST  # Distancia de seguridad mínima (metros)
        self.lidar_subs: Dict[str, Any] = {}  # Diccionario de suscripciones ROS 2 activas
        self.lidar_states: Dict[str, Dict[str, bool]] = {} # Memoria del estado de impacto de cada sensor
        
        self.front_blocked = False
        self.rear_blocked = False

        # Memoria inercial: última velocidad lineal solicitada a las ruedas
        self.last_v = 0.0
        
        # Control del log para no saturar la terminal al frenar
        self._last_print_time = 0.0
        
        # Tarea de fondo: Temporizador para auto-descubrir nuevos LiDARs constantemente
        self.discovery_timer = self.create_timer(2.0, self._discover_lidars)

        self.get_logger().info("[FILTRO] Nodo de seguridad iniciado. Esperando configuración de la app R2Pilot...")

    # =========================================================================
    # LÓGICA ANTICOLISIÓN UNIVERSAL
    # =========================================================================

    def _discover_lidars(self) -> None:
        """!
        @brief Escanea el grafo de ROS 2 buscando publicadores de LaserScan.
        @details Conecta automáticamente el escudo anticolisión a cualquier radar nuevo
                 que se encienda en el robot, aplicando filtros para ignorar tópicos crudos.
        @return None
        """
        topics_and_types = self.get_topic_names_and_types()
        for name, types in topics_and_types:
            
            # =================================================================
            # ¡INTERRUPTOR: MODO ROBOT REAL VS MODO SIMULADOR!
            # Selecciona el perfil adecuado según el entorno de despliegue.
            # =================================================================
            
            # OPCIÓN A) PARA EL ROBOT REAL (Ignora los "_raw" para no verse partes de su propio cuerpo)
            #is_valid_lidar = 'sensor_msgs/msg/LaserScan' in types and name not in self.lidar_subs and 'raw' not in name.lower()
            
            # OPCIÓN B) PARA EL SIMULADOR GAZEBO (Lee todos los láseres sin filtro de cuerpo)
            is_valid_lidar = 'sensor_msgs/msg/LaserScan' in types and name not in self.lidar_subs
            # =================================================================
            
            if is_valid_lidar:
                # Clausura de fábrica para pasar el nombre del tópico al callback
                def make_cb(t_name: str):
                    return lambda msg: self._lidar_callback(msg, t_name)

                # qos_profile_sensor_data relaja la política de fiabilidad para evitar drops de paquetes
                self.lidar_subs[name] = self.create_subscription(LaserScan, name, make_cb(name), qos_profile_sensor_data)
                self.lidar_states[name] = {"front": False, "rear": False}
                self.py_logger.info(f"[DISCOVERY] Escudo Anticolisión anclado automáticamente al sensor: {name}")

    def _lidar_callback(self, msg: LaserScan, topic_name: str) -> None:
        """!
        @brief Analiza matemáticamente una nube de puntos 2D.
        @details Utiliza la proyección del coseno para determinar si los obstáculos
                 están invadiendo el hemisferio frontal o trasero del chasis.
        @param msg Mensaje de ROS 2 con el array de distancias.
        @param topic_name Nombre de la fuente láser analizada.
        @return None
        """
        # Semántica posicional: Identifica si el sensor está montado en la parte trasera físicamente
        is_rear_mounted = 'rear' in topic_name.lower() or 'back' in topic_name.lower()

        front_hit = False
        rear_hit = False

        for i, r in enumerate(msg.ranges):
            # Filtro de ruido: Ignorar valores por debajo de 5cm, infinitos o errores NaN
            if 0.05 < r < self.SAFE_DIST and not math.isinf(r) and not math.isnan(r):
                angle = msg.angle_min + i * msg.angle_increment
                
                # El coseno es positivo (>0) en el semicírculo delantero
                # y negativo (<0) en el semicírculo trasero respecto a la cara del sensor.
                is_front_of_sensor = math.cos(angle) > 0.0

                # Si el sensor está invertido físicamente, invertimos su lógica
                is_robot_front = not is_front_of_sensor if is_rear_mounted else is_front_of_sensor

                if is_robot_front: 
                    front_hit = True
                else: 
                    rear_hit = True

        self.lidar_states[topic_name] = {"front": front_hit, "rear": rear_hit}
        self._evaluate_safety()

    def _evaluate_safety(self) -> None:
        """!
        @brief Compila el estado global de peligro y acciona el actuador de emergencia.
        @details Consolida las lecturas de todos los sensores. Si detecta intención
                 de avance hacia una zona bloqueada, neutraliza la velocidad y emite alertas.
        @return None
        """
        # 1. Buscamos qué sensores exactamente están reportando invasión de su volumen seguro
        front_sensors = [name for name, state in self.lidar_states.items() if state["front"]]
        rear_sensors = [name for name, state in self.lidar_states.items() if state["rear"]]

        self.front_blocked = len(front_sensors) > 0
        self.rear_blocked = len(rear_sensors) > 0

        # 2. FRENO PROACTIVO FÍSICO
        # Solo interviene si hay un obstáculo Y además se intenta avanzar HASTA él (last_v).
        if (self.front_blocked and self.last_v > 0.0) or (self.rear_blocked and self.last_v < 0.0):
            
            # Control de spam de terminal (1 mensaje por segundo máximo)
            if time.time() - self._last_print_time > 1.0:
                self.py_logger.warning(f"[PELIGRO] BLOQUEO PREVENTIVO | Frente: {front_sensors} | Atrás: {rear_sensors}")
                self._last_print_time = time.time()

            self.stop_robot()
            self.last_v = 0.0  # Reseteo inercial para evitar bucles infinitos de frenado

        # 3. Publicación asíncrona del estado hacia el servidor WebSocket (R2Pilot)
        alert_msg = String()
        if self.front_blocked and self.rear_blocked: 
            alert_msg.data = "BOTH"
        elif self.front_blocked: 
            alert_msg.data = "FRONT"
        elif self.rear_blocked: 
            alert_msg.data = "REAR"
        else: 
            alert_msg.data = "OK"
            
        self.alert_pub.publish(alert_msg)

    # =========================================================================
    # LÓGICA DE RUTEO Y PUBLICACIÓN DE VELOCIDAD
    # =========================================================================

    def set_target_topic(self, topic: str) -> None:
        """!
        @brief Reconfigura el destino final de las órdenes de los motores.
        @details Permite al sistema saltar de forma fluida entre el tópico de control
                 de los brazos (joints) y la base móvil (cmd_vel).
        @param topic Cadena de texto con el tópico absoluto de ROS 2.
        @return None
        """
        if self.target_topic == topic and self.real_robot_pub is not None:
            return
            
        self.py_logger.info(f"[RUTEO] Configurando salida de velocidad física hacia: '{topic}'")
        self.target_topic = topic
        
        if self.real_robot_pub is not None:
            self.destroy_publisher(self.real_robot_pub)
            self.real_robot_pub = None 
            
        self.real_robot_pub = self.create_publisher(Twist, self.target_topic, 10)

    def vel_callback(self, msg: Twist) -> None:
        """!
        @brief Callback inyector del filtro matemático sobre las órdenes de joystick.
        @details Aplica las restricciones paramétricas (MAX_V, MAX_W)
                 y aplica las máscaras lógicas generadas por el sistema anticolisión.
        @param msg Mensaje de entrada (Twist crudo) emitido por la interfaz web.
        @return None
        """
        if self.real_robot_pub is None:
            return

        safe_v = msg.linear.x
        safe_w = msg.angular.z
        
        # 1. CORTAFUEGOS DIRECCIONAL
        # Si el usuario empuja adelante (v>0) y hay pared, se cancela la orden lineal a cero.
        if safe_v > 0.0 and self.front_blocked: safe_v = 0.0
        # Si el usuario tira hacia atrás (v<0) y hay pared trasera, se cancela a cero.
        if safe_v < 0.0 and self.rear_blocked: safe_v = 0.0

        # Guardamos la intención validada del usuario para que _evaluate_safety decida frenar si aparece algo de golpe
        self.last_v = safe_v 

        # 2. LIMITACIÓN MATEMÁTICA DE VELOCIDADES
        safe_msg = Twist()
        safe_msg.linear.x = max(min(safe_v, self.MAX_V), -self.MAX_V)
        safe_msg.angular.z = max(min(safe_w, self.MAX_W), -self.MAX_W)
        
        self.real_robot_pub.publish(safe_msg)

    def stop_robot(self) -> None:
        """!
        @brief Dispara una orden incondicional de ceros al controlador local.
        @details Desencadenado por los bloqueos geométricos o por la pérdida de red.
        @return None
        """
        try:
            if rclpy.ok() and self.real_robot_pub is not None:
                self.py_logger.info("[FRENO] Aplicando intervención de emergencia directa al chasis.")
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.real_robot_pub.publish(msg)
        except Exception as e:
            self.py_logger.debug(f"Freno omitido por excepción interna del filtro: {e}")


def main(args=None) -> None:
    """! @brief Punto de entrada y ciclo de vida (Spin) del ejecutable de ROS 2. """
    rclpy.init(args=args)
    node = R2PilotSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()