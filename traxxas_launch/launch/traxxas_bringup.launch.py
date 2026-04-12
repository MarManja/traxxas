import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Lanzamiento de la ZED2i
        Node(
            package='zed_wrapper',
            executable='zed_camera',
            name='zed_camera',
            output='screen',
            parameters=[{'camera_model': 'zed2i'}]
        ),
        
        # Lanzamiento de RPLidar
        Node(
            package='rplidar_ros',
            executable='view_rplidar_a2m12',
            name='rplidar',
            output='screen'
        ),
        
        # Lanzamiento de la pose (IMU y Encoder)
        Node(
            package='traxxas_pose_estimation',
            executable='bringup',
            name='pose_estimation',
            output='screen'
        ),
        
        # Lanzamiento del agente Micro-ROS (revisar parametros)
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'baud_rate': 115200}]
        ),
        
        # Lanzamiento de los sensores ultrasónicos (ajustar)
        #Node(
        #    package='ultrasonic_sensor_package',  # Ajustar el nombre del paquete
        #    executable='ultrasonic_node',  # Ajustar el nombre del nodo
        #    name='ultrasonic_sensors',
        #    output='screen'
        #),
    ])
