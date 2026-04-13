#!/usr/bin/env python3
"""
=======================================================================
 QCarStopRoutineActionServer  -  traxxas_stop
 Adaptado para QCar por: [tu nombre]
 Original (Traxxas/PWM): Marmanja
-----------------------------------------------------------------------
 Action server que gestiona N paradas completas ante señales de ALTO
 para la plataforma QCar física.

 DIFERENCIAS vs versión Traxxas
 ───────────────────────────────
   - Sin PWM. El QCar recibe Vector3Stamped en /qcar/user_command:
       vector.x = throttle  (0.0=stop, 0.04=crucero, 0.06=max)
       vector.y = steering  (-0.3=derecha, 0.0=centro, +0.3=izquierda)
   - Encoder en /qcar/velocity (Vector3Stamped, vector.x = m/s)
     en lugar de /wheel/twist (TwistStamped)
   - StopManeuver usa throttle float en lugar de PWM int
   - La dirección SIEMPRE la sigue controlando el pure_pursuit_node
     via /qcar/user_command. Este servidor solo modifica el throttle
     publicando el mismo topic con el steering que reciba del PP.

 Coordinación con pure_pursuit_node
 ────────────────────────────────────
   stop_active = False → pp_node controla /qcar/user_command (throttle+steering)
   stop_active = True  → ESTE SERVER publica /qcar/user_command
                         con throttle de la maniobra y steering=0.0
                         (el QCar ya está detenido, steering no importa)

 FSM de estados
 ───────────────
   SEARCHING   ← pp_node tiene el control
   APPROACHING ← este server toma el control, reduce throttle por distancia
   BRAKING     ← StopManeuver BRAKING  (rampa + encoder)
   STOPPED     ← StopManeuver STOPPED  (espera 5s)
   RESUMING    ← StopManeuver RESUMING (rampa + encoder)
   COOLDOWN    ← cede control, espera que señal desaparezca

 Tópicos suscritos
 ──────────────────
   /traxxas/stop_event/confirmed   Bool
   /traxxas/stop_event/distance_m  Float32
   /qcar/velocity                  Vector3Stamped  (encoder, vector.x m/s)

 Tópicos publicados
 ───────────────────
   /qcar/user_command              Vector3Stamped  (throttle + steering)
   /traxxas/stop_sign/active       Bool
   /traxxas/mission/done           Bool
=======================================================================
"""

import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, Float32
from traxxas_stop_interfaces.action import StopRoutine
from traxxas_stop.qcar_stop_maneuvers import StopManeuver


class QCarStopRoutineActionServer(Node):

    ST_SEARCHING   = 'SEARCHING'
    ST_APPROACHING = 'APPROACHING'
    ST_BRAKING     = 'BRAKING'
    ST_STOPPED     = 'STOPPED'
    ST_RESUMING    = 'RESUMING'
    ST_COOLDOWN    = 'COOLDOWN'

    LOOP_HZ        = 20
    LOOP_DT        = 1.0 / LOOP_HZ
    COOLDOWN_MIN_S = 2.5

    def __init__(self):
        super().__init__('qcar_stop_routine_action_server')

        self._cb_group = ReentrantCallbackGroup()

        # ── Suscriptores ─────────────────────────────────────────────
        self.create_subscription(
            Bool,    '/traxxas/stop_event/confirmed',
            self._cb_confirmed, 10, callback_group=self._cb_group)

        self.create_subscription(
            Float32, '/traxxas/stop_event/distance_m',
            self._cb_distance, 10, callback_group=self._cb_group)

        # /qcar/velocity — Vector3Stamped, vector.x = velocidad lineal m/s
        self.create_subscription(
            Vector3Stamped, '/qcar/velocity',
            self._cb_velocity, 10, callback_group=self._cb_group)

        # ── Publicadores ─────────────────────────────────────────────
        self._pub_throttle = self.create_publisher(Float32, '/qcar/stop_throttle', 10)

        # ── Estado compartido ─────────────────────────────────────────
        self._confirmed  : bool  = False
        self._distance_m : float = -1.0
        self._velocity   : float = 0.0   # m/s del encoder

        # ── Action Server ─────────────────────────────────────────────
        self._action_server = ActionServer(
            self, StopRoutine, 'stop_routine',
            execute_callback=self._execute_cb,
            callback_group=self._cb_group,
        )

        self.get_logger().info('=' * 55)
        self.get_logger().info(' QCAR STOP ROUTINE ACTION SERVER iniciado')
        self.get_logger().info('  Acción:  /stop_routine')
        self.get_logger().info('  Encoder: /qcar/velocity (Vector3Stamped)')
        self.get_logger().info('  Cmd:     /qcar/user_command (Vector3Stamped)')
        self.get_logger().info('=' * 55)

    # ── Callbacks ─────────────────────────────────────────────────────

    def _cb_confirmed(self, msg: Bool):
        self._confirmed = bool(msg.data)

    def _cb_distance(self, msg: Float32):
        self._distance_m = float(msg.data)

    def _cb_velocity(self, msg: Vector3Stamped):
        # El encoder del QCar reporta velocidad lineal en vector.x
        self._velocity = float(msg.vector.x)

    # ── Helpers de publicación ────────────────────────────────────────

    def _pub_throttle_cmd(self, throttle: float, steering: float = 0.0):
        """
        Publica en /qcar/stop_throttle.
        """
        msg = Float32()
        msg.data = float(throttle)
        self._pub_throttle.publish(msg)

    def _set_active(self, active: bool):
        msg = Bool(); msg.data = active
        self._pub_active.publish(msg)

    def _set_done(self, done: bool):
        msg = Bool(); msg.data = done
        self._pub_done.publish(msg)

    def _send_feedback(self, goal_handle, state, stops_done,
                       throttle_cmd, elapsed_wait=0.0):
        fb = StopRoutine.Feedback()
        fb.state              = state
        fb.stops_completed    = int(stops_done)
        fb.current_pwm        = int(throttle_cmd * 10000)  # escala para compatibilidad
        fb.distance_to_sign_m = float(self._distance_m)
        fb.elapsed_wait_s     = float(elapsed_wait)
        goal_handle.publish_feedback(fb)

    # ── Maquina de estados principal ──────────────────────────────────

    def _execute_cb(self, goal_handle):
        g = goal_handle.request
        total_stops  = int(g.total_stops)
        cruise_cmd   = float(g.cruise_pwm)   # reutilizamos el campo, pero ahora es 0.04
        prepare_dist = float(g.prepare_dist_m)
        stop_dist    = float(g.stop_dist_m)
        wait_time    = float(g.wait_time_s)
        brake_dur    = float(g.brake_duration_s)
        resume_dur   = float(g.resume_duration_s)

        # Si el cliente manda cruise_pwm en valores PWM legacy (>1.0),
        # convertir automáticamente para no romper el cliente existente.
        if cruise_cmd > 1.0:
            self.get_logger().warn(
                f'cruise_pwm={cruise_cmd:.0f} parece valor PWM legacy. '
                f'Usando v_ref=0.04 por defecto.'
            )
            cruise_cmd = 0.04

        self.get_logger().info(
            f'[ACTION] Goal: {total_stops} paradas | '
            f'cruise={cruise_cmd:.3f} | '
            f'prepare={prepare_dist:.2f} m  stop={stop_dist:.2f} m | '
            f'wait={wait_time:.1f} s'
        )

        stops_completed = 0
        state           = self.ST_SEARCHING
        current_throttle = cruise_cmd
        maneuver        = None
        cooldown_start  = 0.0

        self._set_active(False)
        self._set_done(False)

        while rclpy.ok():

            # Cancelación
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('[ACTION] Cancelado por el cliente.')
                self._set_active(False)
                goal_handle.canceled()
                return self._make_result(False, stops_completed, 'Cancelado')

            dist = self._distance_m
            v    = self._velocity
            now  = time.time()

            # ── SEARCHING ────────────────────────────────────────────
            if state == self.ST_SEARCHING:
                self._set_active(False)
                current_throttle = cruise_cmd   # pp_node tiene el control

                if self._confirmed and 0.0 < dist <= prepare_dist:
                    self.get_logger().info(
                        f'[ACTION] Señal a {dist:.2f} m → APPROACHING')
                    state = self.ST_APPROACHING
                    self._set_active(True)

            # ── APPROACHING ──────────────────────────────────────────
            elif state == self.ST_APPROACHING:
                if dist > 0.0:
                    # Interpolación lineal: prepare_dist→cruise, stop_dist→0
                    if dist >= prepare_dist:
                        current_throttle = cruise_cmd
                    elif dist <= stop_dist:
                        current_throttle = 0.0
                    else:
                        t = (dist - stop_dist) / max(prepare_dist - stop_dist, 1e-6)
                        # mínimo de aproximación = 60% del crucero para no parar antes
                        min_approach = cruise_cmd * 0.6
                        current_throttle = min_approach + t * (cruise_cmd - min_approach)

                    self._pub_throttle_cmd(current_throttle)

                    if dist <= stop_dist:
                        self.get_logger().info(
                            f'[ACTION] dist={dist:.3f} m ≤ {stop_dist:.2f} m → BRAKING')
                        state = self.ST_BRAKING
                        maneuver = StopManeuver(
                            cruise_cmd = cruise_cmd,
                            start_cmd  = current_throttle,
                            brake_dur  = brake_dur,
                            wait_s     = wait_time,
                            resume_dur = resume_dur,
                        )
                        maneuver.start()
                else:
                    # Sin distancia válida: avanzar lento
                    current_throttle = cruise_cmd * 0.6
                    self._pub_throttle_cmd(current_throttle)

                # Señal perdida antes de llegar → abortar acercamiento
                if not self._confirmed:
                    self.get_logger().warn(
                        '[ACTION] Señal perdida en APPROACHING → SEARCHING')
                    state = self.ST_SEARCHING
                    self._set_active(False)

            # ── BRAKING / STOPPED / RESUMING ─────────────────────────
            elif state in (self.ST_BRAKING, self.ST_STOPPED, self.ST_RESUMING):
                current_throttle, maneuver_phase = maneuver.tick(v)
                state = maneuver_phase

                if state in (self.ST_BRAKING, self.ST_STOPPED, self.ST_RESUMING):
                    self._pub_throttle_cmd(current_throttle)

                    if state == self.ST_STOPPED:
                        self.get_logger().info(
                            f'[ACTION] Detenido. '
                            f'Esperando {wait_time:.1f}s... '
                            f'({maneuver.wait_elapsed:.1f}s)',
                            throttle_duration_sec=1.0
                        )
                    elif state == self.ST_RESUMING:
                        self.get_logger().info(
                            '[ACTION] Arrancando (rampa)...',
                            throttle_duration_sec=1.0
                        )

                if maneuver.done:
                    stops_completed += 1
                    self.get_logger().info(
                        f'[ACTION] Maniobra completa. '
                        f'Paradas: {stops_completed}/{total_stops} | v={v:.3f} m/s'
                    )
                    state          = self.ST_COOLDOWN
                    cooldown_start = now

            # ── COOLDOWN ─────────────────────────────────────────────
            elif state == self.ST_COOLDOWN:
                self._set_active(False)
                current_throttle = cruise_cmd   # devolver control a pp_node

                elapsed_cd = now - cooldown_start
                sign_gone  = (not self._confirmed) or (dist < 0.0)

                if elapsed_cd >= self.COOLDOWN_MIN_S and sign_gone:
                    if stops_completed >= total_stops:
                        self._set_done(True)
                        self.get_logger().info(
                            f'[ACTION] Misión completa: {stops_completed} paradas.')
                        goal_handle.succeed()
                        return self._make_result(
                            True, stops_completed,
                            f'Misión completada: {stops_completed} señales procesadas.')
                    else:
                        self.get_logger().info(
                            f'[ACTION] Cooldown OK → SEARCHING '
                            f'(faltan {total_stops - stops_completed} paradas)')
                        state = self.ST_SEARCHING

            # ── Feedback y sleep ──────────────────────────────────────
            self._send_feedback(
                goal_handle, state, stops_completed, current_throttle,
                elapsed_wait=maneuver.wait_elapsed if maneuver else 0.0,
            )
            time.sleep(self.LOOP_DT)

        goal_handle.abort()
        return self._make_result(False, stops_completed, 'rclpy shutdown durante ejecución')

    @staticmethod
    def _make_result(success: bool, stops: int, msg: str) -> StopRoutine.Result:
        r = StopRoutine.Result()
        r.success         = success
        r.stops_completed = stops
        r.message         = msg
        return r


def main(args=None):
    rclpy.init(args=args)
    node = QCarStopRoutineActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()