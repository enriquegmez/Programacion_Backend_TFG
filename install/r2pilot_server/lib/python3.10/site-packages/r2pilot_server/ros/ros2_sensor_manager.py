## @file ros2_sensor_manager.py
#  @brief Gestor de telemetría y compresión de sensores.
#  @details Contiene el traductor de ROS 2 a variables Python,
#           controlando la frecuencia de envío hacia el WebSocket.
#  @author Enrique Gómez
#  @date 2026

import time
import logging
from typing import Any, Callable, Dict

from rclpy.qos import qos_profile_sensor_data # type: ignore[import]
from sensor_msgs.msg import BatteryState, LaserScan, Imu, Range, PointCloud2, NavSatFix, Temperature # type: ignore[import]
from nav_msgs.msg import Odometry             # type: ignore[import]
from geometry_msgs.msg import WrenchStamped   # type: ignore[import]

from r2pilot_server.utils.constants import RosMsgTypes, SensorConfig
from r2pilot_server.protocol.models import (  
    Vec3, Quat, Point2D, LaserData, ImuData, BatteryData, RangeData, 
    PointCloudData, OdometryData, NavSatData, WrenchData, TempData, SensorEnvelope
)

class SensorManager:
    """!
    @brief Administrador de streams de sensores para evitar cuellos de botella en la red Wi-Fi.
    @details Se encarga de traducir los mensajes de ROS 2 a variables Python
             y aplicar throttling a 10Hz máximo para no saturar el servidor web.
    """
    def __init__(self, node: Any) -> None:
        """!
        @brief Inicializa memorias del gestor.
        @param node Referencia al nodo principal de ROS 2.
        """
        self.node = node
        self.logger = logging.getLogger("SensorManager")
        self.active_sensor_streams: Dict[str, Dict[str, Any]] = {}

    def start_stream(self, topic: str, callback: Callable) -> bool:
        """!
        @brief Se suscribe a un tópico de ROS 2 según el sensor pedido por el cliente.
        @param topic Nombre del tópico de ROS 2 a suscribirse.
        @param callback Función a ejecutar cuando se recibe un mensaje del tópico.
        @return True si la suscripción fue exitosa, False si hubo un error.
        """
        topics_and_types = self.node.get_topic_names_and_types()
        topic_type = None
        
        for name, types in topics_and_types:
            if name == topic:
                topic_type = types[0] 
                break
                
        if not topic_type:
            self.logger.error(f"No se pudo suscribir: El topic {topic} no existe.")
            return False
            
        if topic in self.active_sensor_streams: return True 

        def make_callback(t: str, s_type: str) -> Callable[[Any], None]:
            return lambda msg: self._sensor_callback(t, msg, s_type)

        sub = None
        if topic_type == RosMsgTypes.LASER_SCAN: sub = self.node.create_subscription(LaserScan, topic, make_callback(topic, "LaserScan"), qos_profile_sensor_data)
        elif topic_type == RosMsgTypes.IMU: sub = self.node.create_subscription(Imu, topic, make_callback(topic, "Imu"), qos_profile_sensor_data)
        elif topic_type == RosMsgTypes.BATTERY: sub = self.node.create_subscription(BatteryState, topic, make_callback(topic, "BatteryState"), qos_profile_sensor_data)
        elif topic_type == RosMsgTypes.RANGE: sub = self.node.create_subscription(Range, topic, make_callback(topic, "Range"), qos_profile_sensor_data)
        elif topic_type == RosMsgTypes.POINT_CLOUD2: sub = self.node.create_subscription(PointCloud2, topic, make_callback(topic, "PointCloud2"), qos_profile_sensor_data)
        elif topic_type == RosMsgTypes.ODOMETRY: sub = self.node.create_subscription(Odometry, topic, make_callback(topic, "Odometry"), qos_profile_sensor_data)
        elif topic_type == RosMsgTypes.NAV: sub = self.node.create_subscription(NavSatFix, topic, make_callback(topic, "NavSatFix"), qos_profile_sensor_data)
        elif topic_type == RosMsgTypes.WRENCH: sub = self.node.create_subscription(WrenchStamped, topic, make_callback(topic, "Wrench"), qos_profile_sensor_data)
        elif topic_type == RosMsgTypes.TEMPERATURE: sub = self.node.create_subscription(Temperature, topic, make_callback(topic, "Temperature"), qos_profile_sensor_data)
        else:
            self.logger.error(f"Tipo de sensor no soportado: {topic_type}")
            return False
            
        self.active_sensor_streams[topic] = {"sub": sub, "callback": callback, "last_sent": 0.0}
        self.logger.info(f"[SENSOR] Suscripción a '{topic}' iniciada.")
        return True

    def stop_stream(self, topic: str) -> None:
        """! 
        @brief Destruye el suscriptor en caliente.
        @param topic Nombre del tópico de ROS 2 a dejar de escuchar.
        """
        if topic in self.active_sensor_streams:
            stream_data = self.active_sensor_streams.pop(topic)
            self.node.destroy_subscription(stream_data["sub"])
            self.logger.info(f"[SENSOR] Suscripción a '{topic}' destruida.")

    def stop_all(self) -> None:
        """! 
        @brief Cierra de golpe todos los streams activos.
        @details Esto es útil cuando el cliente se desconecta o cambia de página.
        """
        topics_to_close = list(self.active_sensor_streams.keys())
        for topic in topics_to_close:
            self.stop_stream(topic)

    def _sensor_callback(self, topic: str, msg: Any, sensor_type: str) -> None:
        """!
        @brief El Gran Traductor de datos crudos a variables python (Throttler a 10Hz máximo).
        @details Este método es llamado por cada suscripción activa de ROS 2.
        @param topic Nombre del tópico de ROS 2 que envía el mensaje.
        @param msg Mensaje de ROS 2 recibido.
        @param sensor_type Tipo de sensor (LaserScan, Imu, BatteryState, etc)
        """
        stream_data = self.active_sensor_streams.get(topic)
        if not stream_data: return
        
        current_time = time.time()
        
        # Obtenemos el tiempo de espera dinámico según el tipo de sensor
        wait_time = SensorConfig.THROTTLE_RATES.get(sensor_type, 0.1)
        
        # Si no ha pasado el tiempo mínimo, descartamos el mensaje (Throttling)
        if current_time - stream_data["last_sent"] < wait_time: return
        
        stream_data["last_sent"] = current_time
        
        payload_data: Any = None
        
        try:
            if sensor_type == "LaserScan":
                step = 3 if len(msg.ranges) > 500 else 1
                max_r = round(float(msg.range_max), 2)
                safe_ranges = [max_r if r == float('inf') or r != r else round(float(r), 2) for r in msg.ranges[::step]]
                payload_data = LaserData(
                    angle_min=msg.angle_min, angle_max=msg.angle_max, angle_increment=msg.angle_increment * step,
                    range_min=msg.range_min, range_max=msg.range_max, ranges=safe_ranges
                )
            elif sensor_type == "Imu":
                payload_data = ImuData(
                    orientation=Quat(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                    angular_velocity=Vec3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
                    linear_acceleration=Vec3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
                )
            elif sensor_type == "BatteryState":
                pct = msg.percentage * 100.0 if msg.percentage <= 1.0 else msg.percentage
                payload_data = BatteryData(voltage=msg.voltage, percentage=pct, power_supply_status=msg.power_supply_status)
            elif sensor_type == "Range":
                payload_data = RangeData(range=msg.range, min_range=msg.min_range, max_range=msg.max_range, field_of_view=msg.field_of_view)
            elif sensor_type == "PointCloud2":
                payload_data = PointCloudData(width=msg.width, height=msg.height, is_dense=msg.is_dense, note="PointCloud2 masivo. Solo metadatos.")
            elif sensor_type == "Odometry":
                payload_data = OdometryData(
                    position=Point2D(msg.pose.pose.position.x, msg.pose.pose.position.y),
                    linear_velocity=msg.twist.twist.linear.x, angular_velocity=msg.twist.twist.angular.z
                )
            elif sensor_type == "NavSatFix":
                payload_data = NavSatData(latitude=msg.latitude, longitude=msg.longitude, altitude=msg.altitude, status=msg.status.status)
            elif sensor_type == "Wrench":
                payload_data = WrenchData(
                    force=Vec3(msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z),
                    torque=Vec3(msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z)
                )
            elif sensor_type == "Temperature":
                payload_data = TempData(temperature=msg.temperature)
                
            # Metemos los datos en el sobre y ejecutamos el callback que nos dio router.py
            envelope = SensorEnvelope(topic=topic, type=sensor_type, data=payload_data)
            stream_data["callback"](envelope)
            
        except Exception as e:
            self.logger.error(f"Error procesando datos del sensor {topic}: {e}")