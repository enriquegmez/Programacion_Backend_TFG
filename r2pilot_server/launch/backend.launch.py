## @file backend_launch.py
#  @brief Archivo de lanzamiento (Launch file) de ROS 2 para el backend R2Pilot.
#  @details Orquesta la ejecución simultánea del servidor asíncrono principal 
#           y de los nodos auxiliares de infraestructura (como web_video_server) 
#           mediante un único comando de terminal.
#  @author Enrique Gómez
#  @date 2026

from launch import LaunchDescription         # type: ignore
from launch_ros.actions import Node          # type: ignore

def generate_launch_description() -> LaunchDescription:
    """!
    @brief Construye la descripción estructural de los nodos a inicializar.
    @details El binario 'ros2 launch' busca obligatoriamente esta función en el script
             para saber qué procesos debe instanciar, qué parámetros inyectarles 
             y cómo gestionar sus salidas por consola.
    @return Instancia de LaunchDescription con la topología de procesos.
    """
    return LaunchDescription([
        
        # =====================================================================
        # 1. NODO PRINCIPAL: Servidor R2Pilot Backend
        # =====================================================================
        # Este nodo llama al entry_point definido en el setup.py, levantando
        # tu main.py (WebSockets, FSM, Controllers, etc.)
        Node(
            package='r2pilot_server',        
            executable='server_node',
            output='screen' # Muestra los logs (INFO, WARNING, ERROR) en la terminal
        ),
        
        # =====================================================================
        # 2. NODO AUXILIAR: Transmisión de Vídeo a Web
        # =====================================================================
        # Convierte los tópicos de ROS 2 (sensor_msgs/Image) en flujos de vídeo
        # MJPEG accesibles por HTTP (usado por la app móvil para ver las cámaras)
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',        
            output='log', 
            parameters=[{'port': 8081}]
        )
    ])