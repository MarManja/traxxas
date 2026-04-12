"""
=======================================================================
 Traxxas 
 Author: Marmanja
-----------------------------------------------------------------------
 Paquetes:
   traxxas_pose_estimation  → pose_traxxas_node  (via bringup.launch.py)
   traxxas_stop             → stop_sign_node
                            → stop_confirmation_node
                            → stop_mission_node
                            → pure_pursuit_stop_traxxas   (pure pursuit + PWM)
                            → traxxas_watchdog_node

 Uso:
   ros2 launch traxxas_stop traxxas_stop.launch.py
   ros2 launch traxxas_stop traxxas_stop.launch.py circle_radius:=1.0 lookahead:=0.2
=======================================================================
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ══════════════════════════════════════════════════════════════
    # ARGUMENTOS
    # ══════════════════════════════════════════════════════════════

    args = [
        # # — Pure Pursuit —
        # DeclareLaunchArgument('circle_radius',    default_value='0.8'),
        # DeclareLaunchArgument('circle_points',    default_value='300'),
        # DeclareLaunchArgument('lookahead',        default_value='0.15'),
        # DeclareLaunchArgument('k_gain',           default_value='0.5'),
        # DeclareLaunchArgument('v_ref',            default_value='0.5'),
        # DeclareLaunchArgument('wheelbase',        default_value='0.324'),
        # DeclareLaunchArgument('path_csv',         default_value='',
        #                       description='Vacío = trayectoria circular'),

        # — PWM Throttle —
        DeclareLaunchArgument('throttle_center',  default_value='2457'),
        DeclareLaunchArgument('throttle_max',     default_value='2600'),
        DeclareLaunchArgument('throttle_min',     default_value='1638'),

        # — PWM Dirección —
        DeclareLaunchArgument('dir_center',       default_value='2642'),
        DeclareLaunchArgument('dir_max_right',    default_value='3276'),
        DeclareLaunchArgument('dir_min_left',     default_value='1669'),
        DeclareLaunchArgument('max_steer_rad',    default_value='0.436'),  # ~25°

        # — Stop Mission —
        DeclareLaunchArgument('prepare_distance', default_value='0.6'),
        DeclareLaunchArgument('brake_distance',   default_value='0.5'),
        DeclareLaunchArgument('stop_distance',    default_value='0.4'),
        DeclareLaunchArgument('wait_seconds',     default_value='5.0'),

        # — Watchdog —
        # DeclareLaunchArgument('watchdog_timeout', default_value='0.3'),

        # — Cámara ZED —
        DeclareLaunchArgument('color_topic',
            default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('depth_topic',
            default_value='/zed/zed_node/depth/depth_registered'),
    ]

    # ══════════════════════════════════════════════════════════════
    # 1 — POSE (incluye el launch del paquete traxxas_pose_estimation)
    # ══════════════════════════════════════════════════════════════

    # pose_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('traxxas_pose_estimation'),
    #             'launch',
    #             'bringup.launch.py'
    #         )
    #     )
    #     # Si bringup.launch.py acepta argumentos puedes pasarlos aquí:
    #     # launch_arguments={'publish_rate': '50.0'}.items()
    # )

    # ══════════════════════════════════════════════════════════════
    # 2 — DETECCIÓN SEÑAL DE ALTO  (paquete: traxxas_stop)
    # ══════════════════════════════════════════════════════════════

    stop_sign_node = Node(
        package='traxxas_stop',
        executable='stop_sign_node',
        name='stop_sign_detector',
        output='screen',
        parameters=[{
            'color_topic': LaunchConfiguration('color_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'min_area':    200.0,
            'debug':       True,
        }]
    )

    # ══════════════════════════════════════════════════════════════
    # 3 — CONFIRMACIÓN POR SCORE  (paquete: traxxas_stop)
    # ══════════════════════════════════════════════════════════════

    stop_confirmation_node = Node(
        package='traxxas_stop',
        executable='stop_confirmation_node',
        name='stop_confirmation',
        output='screen',
        parameters=[{
            'score_gain':                    2,
            'score_decay':                   1,
            'score_max':                     10,
            'score_trigger':                 5,
            'min_area_for_confirmation':     400.0,
            'strong_area_for_confirmation':  5000.0,
            'max_distance_for_confirmation': 0.65,
        }]
    )

    # ══════════════════════════════════════════════════════════════
    # 4 — FSM MISIÓN DE PARADA  (paquete: traxxas_stop)
    # ══════════════════════════════════════════════════════════════

    stop_mission_node = Node(
        package='traxxas_stop',
        executable='stop_mission_node',
        name='stop_mission_manager',
        output='screen',
        parameters=[{
            'prepare_distance': LaunchConfiguration('prepare_distance'),
            'brake_distance':   LaunchConfiguration('brake_distance'),
            'stop_distance':    LaunchConfiguration('stop_distance'),
            'wait_seconds':     LaunchConfiguration('wait_seconds'),
            'cruise_speed':     1.0,
            'approach_speed':   0.5,
            'brake_speed':      0.2,
        }]
    )

    # ══════════════════════════════════════════════════════════════
    # 5 & 6 — PURE PURSUIT + WATCHDOG  (arrancan 2 s después)
    #         El delay da tiempo a que pose_traxxas publique su
    #         primer mensaje antes de que el controlador lo necesite.
    # ══════════════════════════════════════════════════════════════

    # delayed_nodes = TimerAction(
    #     period=2.0,
    #     actions=[
    #         LogInfo(msg='[launch] Pose lista → arrancando Pure Pursuit y Watchdog'),

    #         # — Pure Pursuit —
    #         Node(
    #             package='traxxas_stop',
    #             executable='pure_pursuit_stop_traxxas',   # nombre en setup.py
    #             name='pure_pursuit_traxxas',
    #             output='screen',
    #             parameters=[{
    #                 'path_csv':        LaunchConfiguration('path_csv'),
    #                 'circle_radius':   LaunchConfiguration('circle_radius'),
    #                 'circle_points':   LaunchConfiguration('circle_points'),
    #                 'lookahead':       LaunchConfiguration('lookahead'),
    #                 'k_gain':          LaunchConfiguration('k_gain'),
    #                 'v_ref':           LaunchConfiguration('v_ref'),
    #                 'wheelbase':       LaunchConfiguration('wheelbase'),
    #                 'throttle_center': LaunchConfiguration('throttle_center'),
    #                 'throttle_max':    LaunchConfiguration('throttle_max'),
    #                 'throttle_min':    LaunchConfiguration('throttle_min'),
    #                 'dir_center':      LaunchConfiguration('dir_center'),
    #                 'dir_max_right':   LaunchConfiguration('dir_max_right'),
    #                 'dir_min_left':    LaunchConfiguration('dir_min_left'),
    #                 'max_steer_rad':   LaunchConfiguration('max_steer_rad'),
    #             }]
    #         ),

            # — Watchdog —
            # Node(
            #     package='traxxas_stop',
            #     executable='traxxas_watchdog_node',
            #     name='traxxas_watchdog',
            #     output='screen',
            #     parameters=[{
            #         'timeout':         LaunchConfiguration('watchdog_timeout'),
            #         'throttle_center': LaunchConfiguration('throttle_center'),
            #         'throttle_min':    LaunchConfiguration('throttle_min'),
            #         'dir_center':      LaunchConfiguration('dir_center'),
            #     }]
            # ),
    #     ]
    # )

    # ══════════════════════════════════════════════════════════════
    # LAUNCH DESCRIPTION
    # ══════════════════════════════════════════════════════════════

    return LaunchDescription(
        args + [
            # pose_launch,            # traxxas_pose_estimation/bringup.launch.py
            stop_sign_node,         # detección ZED2i
            stop_confirmation_node, # filtro score
            stop_mission_node,      # FSM parada
            # delayed_nodes,          # pure pursuit + watchdog (t+2s)
        ]
    )