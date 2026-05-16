"""
safety_filter.py
Nodo independiente de ROS 2.
Escucha comandos crudos de la app web, filtra velocidades peligrosas y publica al robot.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import logging

class SafetyFilterNode(Node):
    def __init__(self):
        super().__init__('app_safety_filter')
        
        self.py_logger = logging.getLogger("SafetyFilterNode")

        # 1. Nos suscribimos a los datos crudos que manda el servidor web
        self.raw_sub = self.create_subscription(
            Twist, 
            'web_teleop/cmd_vel_raw', 
            self.vel_callback, 
            10
        )
        
# 1. LAZY INITIALIZATION: Empezamos sin publicador ni topic
        self.target_topic = None
        self.real_robot_pub = None
        
        # LÍMITES DE SEGURIDAD
        self.MAX_V = 0.5  # m/s
        self.MAX_W = 1.0  # rad/s
        
        self.get_logger().info("Filtro de seguridad iniciado. Esperando configuración del cliente...")

    def set_target_topic(self, topic: str):
        """Crea o reconfigura el publicador cuando el cliente lo solicita."""
        # Si ya estábamos publicando en este topic, nos ahorramos el trabajo
        if self.target_topic == topic and self.real_robot_pub is not None:
            return
            
        #self.get_logger().info(f"Configurando salida de velocidad hacia: {topic}")
        self.py_logger.info(f"--> [ÉXITO] Configurando salida de velocidad hacia: '{topic}'")
        self.target_topic = topic
        
        # Si por algún casual ya existía uno (el cliente cambió de opinión en pleno vuelo), lo limpiamos
        if self.real_robot_pub is not None:
            self.py_logger.info("Destruyendo enlace DDS del publicador anterior...")
            self.destroy_publisher(self.real_robot_pub)
            self.real_robot_pub = None # <--- CRÍTICO: Forzamos a Python a vaciar la memoria
            
        # Creamos el publicador definitivo
        self.real_robot_pub = self.create_publisher(Twist, self.target_topic, 10) #PONER CUANDO ESTE BIEN EN FROTNEND
        #self.real_robot_pub = self.create_publisher(Twist, 'mobile_base_controller/cmd_vel_unstamped', 10)

    def vel_callback(self, msg: Twist):
        """Recibe el comando del web, lo limita por seguridad y lo envía al robot."""
        # Protección vital: Si nos llegan velocidades pero aún no se ha creado el publicador, las ignoramos
        if self.real_robot_pub is None:
            return

        safe_msg = Twist()
        
        safe_msg.linear.x = max(min(msg.linear.x, self.MAX_V), -self.MAX_V)
        safe_msg.angular.z = max(min(msg.angular.z, self.MAX_W), -self.MAX_W)
        
        self.real_robot_pub.publish(safe_msg)

    def stop_robot(self):
        """Freno directo de emergencia al tópico real del robot."""
        try:
            if rclpy.ok() and self.real_robot_pub is not None:
                self.py_logger.info("SafetyFilter: Aplicando freno de emergencia directo a las ruedas...")
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.real_robot_pub.publish(msg)
        except Exception as e:
            self.py_logger.debug(f"Freno omitido en SafetyFilter: {e}")


#POR SI QUIERO EJECUTAR ESTE NODO POR SEPARADO PARA PRUEBAS
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