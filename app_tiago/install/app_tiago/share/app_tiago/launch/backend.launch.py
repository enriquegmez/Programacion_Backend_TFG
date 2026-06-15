import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # 1. Tu servidor principal (El que orquesta todo)
        # Nota: Sustituye 'server_node' por el nombre exacto que le pusiste
        # en tu setup.py (console_scripts)
        Node(
            package='app_tiago',
            executable='server_node',
            name='tiago_backend_server',
            output='screen' # Para ver tus logs (info, warnings) en la terminal
        ),
        
        # 2. El servidor de vídeo en segundo plano
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='log', # Ocultamos sus logs para que no ensucien tus mensajes
            parameters=[{'port': 8081}]
        )
    ])