"""
=======================================================================
 QCar Full Stack Launch — Señal de Alto + Pure Pursuit
=======================================================================
 Nodos que levanta (en orden con delays):
   t=0s  1. ic_rgbd           (vision_helpers_pkg) — descomprime cámara RGBD
   t=0s  2. pose_estimator     (sensores_kalman)    — EKF encoder + IMU
   t=0s  3. qcar_watchdog      (sensores_kalman)    — watchdog de seguridad
   t=2s  4. qcar_stop_sign     (traxxas_stop)       — visión: detecta señal
   t=3s  5. stop_confirmation  (traxxas_stop)       — filtro score temporal
   t=3s  6. stop_event_merger  (traxxas_stop)       — fusiona stop+crosswalk
   t=3s  7. qcar_pp_node       (traxxas_stop)       — Pure Pursuit
   t=4s  8. stop_routine_server(traxxas_stop)       — action server de parada

 El cliente (stop_routine_client) se lanza manualmente en otra terminal:
   ros2 run traxxas_stop qcar_stop_routine_client

 Flujo de tópicos
 ──────────────────
   /qcar/rgbd_color
     └─► ic_rgbd ──► /qcar/decompressed/rgbd_color
                          └─► qcar_stop_sign_node
                                ├─► /traxxas/stop_sign/detected_raw
                                ├─► /traxxas/stop_sign/area
                                └─► /traxxas/stop_sign/distance_m
                                        └─► stop_confirmation_node
                                              └─► /traxxas/stop_sign/confirmed
                                                      └─► stop_event_merger
                                                            └─► /traxxas/stop_event/confirmed
                                                            └─► /traxxas/stop_event/distance_m
                                                                    └─► stop_routine_server
                                                                          └─► /traxxas/stop_sign/active
                                                                                └─► qcar_pp_node
   /qcar/velocity ──► stop_routine_server (encoder feedback)
   /qcar/pose     ──► qcar_pp_node
   /qcar/user_command ◄── qcar_pp_node  (throttle+steering, normal)
   /qcar/user_command ◄── stop_routine_server (throttle, durante parada)
=======================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Argumentos ───────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('path_csv', default_value='',
            description='Ruta al CSV con trayectoria (x,y). Vacío = círculo.'),
        DeclareLaunchArgument('v_ref', default_value='0.06',
            description='Throttle crucero QCar (0.0–0.06)'),
        DeclareLaunchArgument('total_stops', default_value='3',
            description='Número de señales de alto a procesar'),
        DeclareLaunchArgument('prepare_dist_m', default_value='0.60',
            description='Distancia de inicio de aproximación [m]'),
        DeclareLaunchArgument('stop_dist_m', default_value='0.30',
            description='Distancia de parada completa [m]'),
        DeclareLaunchArgument('wait_time_s', default_value='5.0',
            description='Segundos detenido ante la señal'),
        DeclareLaunchArgument('brake_duration_s', default_value='2.0',
            description='Segundos de rampa de frenado'),
        DeclareLaunchArgument('resume_duration_s', default_value='1.5',
            description='Segundos de rampa de arranque'),
        DeclareLaunchArgument('debug_vision', default_value='true',
            description='Publica /traxxas/stop_sign/debug_image'),
    ]

    # ── t=0s: Descompresor de cámara RGBD ────────────────────────────
    image_decompressor = Node(
        package='vision_helpers_pkg',
        executable='img_converter',
        name='ic_rgbd',
        output='screen',
        parameters=[
            {'subscribe_topic': '/qcar/rgbd_color'},
            {'publish_topic':   '/qcar/decompressed/rgbd_color'},
        ]
    )

    # ── t=0s: Estimador de pose ───────────────────────────────────────
    pose_node = Node(
        package='sensores_kalman',
        executable='pose_ekf_qcar_2',
        name='pose_estimator',
        output='screen',
    )

    # # ── t=0s: Watchdog de seguridad ───────────────────────────────────
    # watchdog_node = Node(
    #     package='sensores_kalman',
    #     executable='qcar_watchdog_node',
    #     name='qcar_watchdog',
    #     output='screen',
    #     parameters=[{'timeout': 0.2}]
    # )

    # ── t=2s: Detector visual de señal de alto ───────────────────────
    stop_sign_node = TimerAction(period=2.0, actions=[
        Node(
            package='traxxas_stop',
            executable='qcar_stop_sign_node',
            name='stop_sign_detector',
            output='screen',
            parameters=[
                {'color_topic': '/qcar/decompressed/rgbd_color'},
                {'depth_topic': '/qcar/rgbd_depth'},
                {'min_area':    200.0},
                {'debug':       LaunchConfiguration('debug_vision')},
            ]
        )
    ])

    # ── t=3s: Filtro de confirmación temporal (score) ─────────────────
    stop_confirmation_node = TimerAction(period=3.0, actions=[
        Node(
            package='traxxas_stop',
            executable='stop_confirmation_node',
            name='stop_confirmation',
            output='screen',
            parameters=[
                {'score_gain':                    2},
                {'score_decay':                   1},
                {'score_max':                    10},
                {'score_trigger':                 5},
                {'min_area_for_confirmation':   400.0},
                {'strong_area_for_confirmation': 5000.0},
                {'max_distance_for_confirmation': 0.65},
            ]
        )
    ])

    # ── t=3s: Merger stop_sign + crosswalk → stop_event ──────────────
    stop_event_merger_node = TimerAction(period=3.0, actions=[
        Node(
            package='traxxas_stop',
            executable='stop_event_merger_node',
            name='stop_event_merger',
            output='screen',
        )
    ])

    # ── t=3s: Pure Pursuit ────────────────────────────────────────────
    pure_pursuit_node = TimerAction(period=3.0, actions=[
        Node(
            package='traxxas_stop',
            executable='qcar_pp_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[
                {'path_csv':      LaunchConfiguration('path_csv')},
                {'v_ref':         LaunchConfiguration('v_ref')},
                {'lookahead':     0.1},
                {'k_gain':        0.8},
                {'wheelbase':     0.256},
                {'circle_radius': 0.8},
                {'circle_points': 300},
            ]
        )
    ])

    # ── t=4s: Action Server de parada ────────────────────────────────
    # Se lanza último porque necesita que el PP ya esté publicando
    stop_routine_server = TimerAction(period=4.0, actions=[
        Node(
            package='traxxas_stop',
            executable='qcar_stop_routine_server',
            name='qcar_stop_routine_action_server',
            output='screen',
        )
    ])

    return LaunchDescription(args + [
        image_decompressor,
        pose_node,
        # watchdog_node,
        stop_sign_node,
        stop_confirmation_node,
        stop_event_merger_node,
        pure_pursuit_node,
        stop_routine_server,
    ])