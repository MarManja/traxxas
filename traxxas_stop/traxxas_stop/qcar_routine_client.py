#!/usr/bin/env python3
"""
Cliente de la action StopRoutine.
Lanza la misión de 3 señales de alto y reporta feedback en terminal.

Uso
---
  # Terminal 1: lanzar todo el stack (sensores + detección + pp_node)
  ros2 launch traxxas_stop bringup_stop.launch.py

  # Terminal 2: correr el action server
  ros2 run traxxas_stop stop_routine_action_server

  # Terminal 3: enviar el goal con este cliente
  ros2 run traxxas_stop stop_routine_client

O bien ajustar los parámetros del goal directamente aquí abajo.
"""

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from traxxas_stop_interfaces.action import StopRoutine


class StopRoutineClient(Node):

    def __init__(self):
        super().__init__('stop_routine_client')
        self._client = ActionClient(self, StopRoutine, 'stop_routine')

    def send_goal(self):
        self.get_logger().info('Esperando action server...')
        self._client.wait_for_server()

        # ── Configurar la misión ──────────────────────────────
        goal = StopRoutine.Goal()
        goal.total_stops      = 3        # 3 señales de alto
        goal.cruise_pwm       = 0.04  # PWM velocidad crucero
        goal.brake_pwm        = 0.0   # PWM freno completo
        goal.prepare_dist_m   = 0.80     # Inicio de aproximación (m)
        goal.stop_dist_m      = 0.30     # Máximo 30 cm de la señal
        goal.wait_time_s      = 5.0      # Esperar 5 segundos detenido
        goal.brake_duration_s = 2.0      # Frenado gradual en 2 segundos
        goal.resume_duration_s = 1.5     # Arranque gradual en 1.5 segundos

        self.get_logger().info(
            f'Enviando goal: {goal.total_stops} paradas | '
            f'cruise={goal.cruise_pwm:.0f} | '
            f'stop_dist={goal.stop_dist_m:.2f} m'
        )

        send_goal_future = self._client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rechazado por el servidor.')
            return
        self.get_logger().info('Goal aceptado. Misión en progreso...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[{fb.state:12s}] '
            f'paradas={fb.stops_completed} | '
            f'PWM={fb.current_pwm:4d} | '
            f'dist={fb.distance_to_sign_m:.2f} m | '
            f'espera={fb.elapsed_wait_s:.1f} s'
        )

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f' Misión COMPLETADA: {result.stops_completed} paradas. '
                f'{result.message}'
            )
        else:
            self.get_logger().error(
                f' Misión FALLIDA: {result.message}'
            )
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    client = StopRoutineClient()
    client.send_goal()
    rclpy.spin(client)


if __name__ == '__main__':
    main()

