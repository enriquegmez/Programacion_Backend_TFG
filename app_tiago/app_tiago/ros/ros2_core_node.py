"""
safety_filter.py
Nodo independiente de ROS 2.
Escucha comandos crudos de la app web, filtra velocidades peligrosas y publica al robot.
"""

import rclpy # type: ignore[import]
from rclpy.node import Node # type: ignore[import]
from geometry_msgs.msg import Twist # type: ignore[import]
from sensor_msgs.msg import LaserScan # type: ignore[import]
from std_msgs.msg import String # type: ignore[import]
from rclpy.qos import qos_profile_sensor_data # type: ignore[import]
import logging
import math

class SafetyFilterNode(Node):
    def __init__(self):
        super().__init__('app_safety_filter')
        
        self.py_logger = logging.getLogger("SafetyFilterNode")

        # 1. Suscripción al Joystick Web
        self.raw_sub = self.create_subscription(Twist, 'web_teleop/cmd_vel_raw', self.vel_callback, 10)
        
        # 2. Canal de alertas para avisar al móvil
        self.alert_pub = self.create_publisher(String, 'web_teleop/safety_alert', 10)
        
        self.target_topic = None
        self.real_robot_pub = None
        
        # LÍMITES DE VELOCIDAD
        self.MAX_V = 0.5  # m/s
        self.MAX_W = 1.0  # rad/s
        
        # ==========================================
        # ¡NUEVO! VARIABLES DEL ESCUDO ANTICOLISIÓN
        # ==========================================
        self.SAFE_DIST = 0.5 # Distancia mínima en metros
        self.lidar_subs = {}
        self.lidar_states = {}
        
        self.front_blocked = False
        self.rear_blocked = False

        # ¡NUEVO! Memoria de la última velocidad enviada
        self.last_v = 0.0
        
        # Temporizador para auto-descubrir cualquier LiDAR que se encienda en el robot
        self.discovery_timer = self.create_timer(2.0, self._discover_lidars)

        self.get_logger().info("Filtro de seguridad iniciado. Esperando configuración del cliente...")

    # ==========================================
    # LÓGICA ANTICOLISIÓN UNIVERSAL (Trigonometría)
    # ==========================================
    def _discover_lidars(self):
        topics_and_types = self.get_topic_names_and_types()
        for name, types in topics_and_types:
            
            # =========================================================================
            # ¡INTERRUPTOR: MODO ROBOT REAL VS MODO SIMULADOR!
            # =========================================================================
            # Descomenta la línea que necesites según dónde estés ejecutando el backend.
            
            # OPCIÓN A) PARA EL ROBOT REAL (Ignora los "_raw" para no verse a sí mismo)
            #es_lidar_valido = 'sensor_msgs/msg/LaserScan' in types and name not in self.lidar_subs and 'raw' not in name.lower()
            
            # OPCIÓN B) PARA EL SIMULADOR GAZEBO (Lee todos los láseres)
            es_lidar_valido = 'sensor_msgs/msg/LaserScan' in types and name not in self.lidar_subs
            # =========================================================================

            if es_lidar_valido:
                # Fábrica para saber de qué LiDAR viene la lectura
                def make_cb(t_name):
                    return lambda msg: self._lidar_callback(msg, t_name)

                # Usamos qos_profile_sensor_data para evitar Warnings de incompatibilidad
                self.lidar_subs[name] = self.create_subscription(LaserScan, name, make_cb(name), qos_profile_sensor_data)
                self.lidar_states[name] = {"front": False, "rear": False}
                self.py_logger.info(f"🛡️ Escudo Anticolisión anclado automáticamente al sensor limpio: {name}")

    def _lidar_callback(self, msg: LaserScan, topic_name: str):
        # Una pequeña ayuda semántica por si el robot tiene sensores montados del revés físicamente
        is_rear_mounted = 'rear' in topic_name.lower() or 'back' in topic_name.lower()

        front_hit = False
        rear_hit = False

        for i, r in enumerate(msg.ranges):
            # Ignoramos rebotes internos (<0.05m) y fallos del sensor (inf o NaN)
            if 0.05 < r < self.SAFE_DIST and not math.isinf(r) and not math.isnan(r):
                angle = msg.angle_min + i * msg.angle_increment
                
                # ¡LA MAGIA MATEMÁTICA! El coseno nos dice si el punto está en la mitad delantera (>0) o trasera (<0)
                is_front_of_sensor = math.cos(angle) > 0.0

                # Si el sensor está montado al revés, invertimos la lógica
                is_robot_front = not is_front_of_sensor if is_rear_mounted else is_front_of_sensor

                if is_robot_front: front_hit = True
                else: rear_hit = True

        self.lidar_states[topic_name] = {"front": front_hit, "rear": rear_hit}
        self._evaluate_safety()

    def _evaluate_safety(self):
        # 1. Buscamos qué sensores exactamente están dando la alarma
        sensores_frente = [name for name, state in self.lidar_states.items() if state["front"]]
        sensores_atras = [name for name, state in self.lidar_states.items() if state["rear"]]

        self.front_blocked = len(sensores_frente) > 0
        self.rear_blocked = len(sensores_atras) > 0

        # ==========================================
        # ¡EL FRENO PROACTIVO FÍSICO!
        # ==========================================
        # Solo entramos aquí si hay un obstáculo Y además intentamos avanzar hacia él (last_v)
        if (self.front_blocked and self.last_v > 0.0) or (self.rear_blocked and self.last_v < 0.0):
            
            # ¡EL CHIVATO SILENCIOSO! Solo imprimimos 1 vez por segundo y SOLO cuando frenamos el joystick
            import time
            if not hasattr(self, '_last_print_time') or time.time() - self._last_print_time > 1.0:
                self.py_logger.warning(f"🛑 BLOQUEO PREVENTIVO | Obstáculo Frente: {sensores_frente} | Atrás: {sensores_atras}")
                self._last_print_time = time.time()

            self.stop_robot()
            self.last_v = 0.0 # Reseteamos para no enviar frenos en bucle infinito

        # Publicamos el estado al móvil
        alert_msg = String()
        if self.front_blocked and self.rear_blocked: alert_msg.data = "BOTH"
        elif self.front_blocked: alert_msg.data = "FRONT"
        elif self.rear_blocked: alert_msg.data = "REAR"
        else: alert_msg.data = "OK"
        self.alert_pub.publish(alert_msg)

    # ==========================================
    # LÓGICA DE PUBLICACIÓN DE VELOCIDAD
    # ==========================================
    def set_target_topic(self, topic: str):
        if self.target_topic == topic and self.real_robot_pub is not None:
            return
            
        self.py_logger.info(f"--> [ÉXITO] Configurando salida de velocidad hacia: '{topic}'")
        self.target_topic = topic
        
        if self.real_robot_pub is not None:
            self.destroy_publisher(self.real_robot_pub)
            self.real_robot_pub = None 
            
        self.real_robot_pub = self.create_publisher(Twist, self.target_topic, 10)

    def vel_callback(self, msg: Twist):
        if self.real_robot_pub is None:
            return

        safe_v = msg.linear.x
        safe_w = msg.angular.z
        

        # ¡EL CORTAFUEGOS FÍSICO!
        # Si vas hacia adelante (v>0) y el frente está bloqueado, fuerza la velocidad a 0
        if safe_v > 0.0 and self.front_blocked: safe_v = 0.0
        # Si vas hacia atrás (v<0) y la trasera está bloqueada, fuerza la velocidad a 0
        if safe_v < 0.0 and self.rear_blocked: safe_v = 0.0

        self.last_v = safe_v # ¡Guardamos lo que estamos enviando a las ruedas!

        safe_msg = Twist()
        safe_msg.linear.x = max(min(safe_v, self.MAX_V), -self.MAX_V)
        safe_msg.angular.z = max(min(safe_w, self.MAX_W), -self.MAX_W)
        
        self.real_robot_pub.publish(safe_msg)

    def stop_robot(self):
        try:
            if rclpy.ok() and self.real_robot_pub is not None:
                self.py_logger.info("SafetyFilter: Aplicando freno de emergencia directo a las ruedas...")
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.real_robot_pub.publish(msg)
        except Exception as e:
            self.py_logger.debug(f"Freno omitido en SafetyFilter: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SafetyFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()