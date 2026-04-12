#!/usr/bin/env python3
"""
=======================================================================
 StopRoutineActionServer  -  traxxas_stop
 Autor: Marmanja
-----------------------------------------------------------------------
 Action server que gestiona N paradas completas ante senales de ALTO.

 Coordinacion con pp_node
 ------------------------
   stop_active = False  ->  pp_node controla /throttle_motor
   stop_active = True   ->  ESTE SERVER controla /throttle_motor
   Direccion (/direction_servo) SIEMPRE la controla pp_node.

 Retroalimentacion del encoder
   Suscrito a /wheel/twist (TwistStamped, linear.x en m/s).
   - BRAKING  : StopManeuver/BrakeRamp extiende la rampa hasta
                que |v| < 0.03 m/s (encoder). Evita declarar
                "detenido" si el vehiculo aun se mueve.
   - STOPPED  : solo temporal (5 s por defecto).
   - RESUMING : StopManeuver/ResumeRamp espera v > 0.10 m/s tras
                la rampa de tiempo. Evita pasar a COOLDOWN si el
                vehiculo no arranco realmente.

 FSM de estados
 ---------------
   SEARCHING   <- pp_node tiene throttle
   APPROACHING <- este server toma throttle, reduce PWM por distancia ZED
   BRAKING     <- StopManeuver BRAKING  (rampa + encoder confirm)
   STOPPED     <- StopManeuver STOPPED  (tiempo)
   RESUMING    <- StopManeuver RESUMING (rampa + encoder confirm)
   COOLDOWN    <- cede throttle, espera sennal desaparezca

 Topicos suscritos
 -----------------
   /traxxas/stop_sign/confirmed   Bool
   /traxxas/stop_sign/distance_m  Float32
   /wheel/twist                   TwistStamped  (encoder)

 Topicos publicados
 ------------------
   /throttle_motor                String (uint16 PWM)
   /traxxas/stop_sign/active      Bool
   /traxxas/mission/done          Bool
=======================================================================
"""

import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Float32, String
from traxxas_stop_interfaces.action import StopRoutine
from traxxas_stop.pwm_controller import PWMController
from traxxas_stop.stop_maneuvers import StopManeuver


class StopRoutineActionServer(Node):
    ST_SEARCHING   = 'SEARCHING'
    ST_APPROACHING = 'APPROACHING'
    ST_BRAKING     = 'BRAKING'
    ST_STOPPED     = 'STOPPED'
    ST_RESUMING    = 'RESUMING'
    ST_COOLDOWN    = 'COOLDOWN'

    LOOP_HZ        = 20
    LOOP_DT        = 1.0 / LOOP_HZ
    COOLDOWN_MIN_S = 2.5   # tiempo minimo de cooldown entre paradas

    def __init__(self):
        super().__init__('stop_routine_action_server')

        self._cb_group = ReentrantCallbackGroup()

        self.create_subscription(Bool, '/traxxas/stop_event/confirmed',self._cb_confirmed, 10, callback_group=self._cb_group)
        self.create_subscription(Float32, '/traxxas/stop_event/distance_m',self._cb_distance, 10, callback_group=self._cb_group)
        self.create_subscription(TwistStamped, '/wheel/twist',self._cb_twist, 10, callback_group=self._cb_group)

        self._pub_throttle = self.create_publisher(String, '/throttle_motor',          10)
        self._pub_active   = self.create_publisher(Bool,   '/traxxas/stop_sign/active', 10) 
        self._pub_done     = self.create_publisher(Bool,   '/traxxas/mission/done',     10)

        # Estado compartido (actualizado por callbacks) 
        self._confirmed  : bool  = False
        self._distance_m : float = -1.0
        self._velocity   : float = 0.0   # m/s del encoder

        # Action Server 
        self._action_server = ActionServer(self, StopRoutine, 'stop_routine',execute_callback=self._execute_cb,callback_group=self._cb_group,)

        self.get_logger().info('STOP ROUTINE ACTION SERVER iniciado')
        self.get_logger().info('  Accion: /stop_routine')
        self.get_logger().info('  Encoder: /wheel/twist')

    #  CALLBACKS
    def _cb_confirmed(self, msg: Bool):
        self._confirmed = bool(msg.data)

    def _cb_distance(self, msg: Float32):
        self._distance_m = float(msg.data)

    def _cb_twist(self, msg: TwistStamped):
        self._velocity = float(msg.twist.linear.x)


    #  HELPERS DE PUBLICACION
    def _pub_pwm(self, pwm: int):
        msg = String(); msg.data = str(int(pwm))
        self._pub_throttle.publish(msg)

    def _set_active(self, active: bool):
        msg = Bool(); msg.data = active
        self._pub_active.publish(msg)

    def _set_done(self, done: bool):
        msg = Bool(); msg.data = done
        self._pub_done.publish(msg)

    def _send_feedback(self, goal_handle, state, stops_done, pwm, elapsed_wait=0.0):
        fb = StopRoutine.Feedback()
        fb.state              = state
        fb.stops_completed    = int(stops_done)
        fb.current_pwm        = int(pwm)
        fb.distance_to_sign_m = float(self._distance_m)
        fb.elapsed_wait_s     = float(elapsed_wait)
        goal_handle.publish_feedback(fb)


    #  EXECUTE CALLBACK - maquina de estados principal
    def _execute_cb(self, goal_handle):
        """
        Bucle principal de la action.
        Usa PWMController para APPROACHING y StopManeuver para
        BRAKING/STOPPED/RESUMING con retroalimentacion del encoder.
        """
        g = goal_handle.request
        total_stops  = int(g.total_stops)
        cruise_pwm   = int(g.cruise_pwm)
        brake_pwm    = int(g.brake_pwm)
        prepare_dist = float(g.prepare_dist_m)
        stop_dist    = float(g.stop_dist_m)
        wait_time    = float(g.wait_time_s)
        brake_dur    = float(g.brake_duration_s)
        resume_dur   = float(g.resume_duration_s)

        # Controlador PWM para calcular throttle segun distancia (APPROACHING)
        pwm_ctrl = PWMController(
            brake_pwm  = brake_pwm,
            cruise_pwm = cruise_pwm,
            max_pwm    = cruise_pwm, # max_pwm opcional
        )

        self.get_logger().info(
            f'[ACTION] Goal: {total_stops} paradas | '
            f'cruise={cruise_pwm} brake={brake_pwm} | '
            f'prepare={prepare_dist:.2f} m stop={stop_dist:.2f} m | '
            f'wait={wait_time:.1f} s'
        )

        stops_completed = 0
        state           = self.ST_SEARCHING
        current_pwm     = cruise_pwm
        maneuver        = None     # StopManeuver activo
        cooldown_start  = 0.0

        self._set_active(False)
        self._set_done(False)

        while rclpy.ok():
            # Cancelacion solicitada por el cliente
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('[ACTION] Cancelado por el cliente.')
                self._set_active(False)
                goal_handle.canceled()
                return self._make_result(False, stops_completed, 'Cancelado por el cliente')

            dist = self._distance_m
            v    = self._velocity
            now  = time.time()

            # ----------------------------------------------------------
            # SEARCHING: pp_node controla el throttle
            # ----------------------------------------------------------
            if state == self.ST_SEARCHING:
                self._set_active(False)
                current_pwm = cruise_pwm

                if self._confirmed and 0.0 < dist <= prepare_dist:
                    self.get_logger().info(
                        f'[ACTION] Senal detectada a {dist:.2f} m -> APPROACHING')
                    state = self.ST_APPROACHING
                    self._set_active(True)   # toma del throttle

            # ----------------------------------------------------------
            # APPROACHING: reducir PWM proporcional a distancia (ZED)
            # ----------------------------------------------------------
            elif state == self.ST_APPROACHING:
                if dist > 0.0:
                    current_pwm = pwm_ctrl.throttle_from_distance(
                        dist, stop_dist, prepare_dist, approach_pwm_offset=40)
                    self._pub_pwm(current_pwm)

                    if dist <= stop_dist:
                        self.get_logger().info(
                            f'[ACTION] dist={dist:.3f} m <= {stop_dist:.2f} m -> BRAKING')
                        state = self.ST_BRAKING
                        maneuver = StopManeuver(
                            start_pwm  = current_pwm,
                            brake_pwm  = brake_pwm,
                            cruise_pwm = cruise_pwm,
                            brake_dur  = brake_dur,
                            wait_s     = wait_time,
                            resume_dur = resume_dur,
                        )
                        maneuver.start()
                else:
                    # ZED no da distancia valida: acercamiento lento
                    current_pwm = brake_pwm + 40
                    self._pub_pwm(current_pwm)

                # Senal perdida antes de llegar -> abortar acercamiento
                if not self._confirmed:
                    self.get_logger().warn(
                        '[ACTION] Senal perdida en APPROACHING -> SEARCHING')
                    state = self.ST_SEARCHING
                    self._set_active(False)

            # ----------------------------------------------------------
            # BRAKING / STOPPED / RESUMING via StopManeuver
            # El tick recibe la velocidad del encoder para retroalimentar
            # ----------------------------------------------------------
            elif state in (self.ST_BRAKING, self.ST_STOPPED, self.ST_RESUMING):
                current_pwm, maneuver_phase = maneuver.tick(v)
                state = maneuver_phase   # sincronia con la maniobra

                if state in (self.ST_BRAKING, self.ST_STOPPED, self.ST_RESUMING):
                    self._pub_pwm(current_pwm)

                    # Logs de transicion
                    if state == self.ST_STOPPED and maneuver.phase == StopManeuver.STOPPED:
                        pass   # la transicion ya fue logueada por BrakeRamp
                    if state == self.ST_RESUMING and maneuver.phase == StopManeuver.RESUMING:
                        self.get_logger().info(
                            f'[ACTION] Detencion confirmada (encoder). Arrancando...')

                if maneuver.done:
                    stops_completed += 1
                    self.get_logger().info(
                        f'[ACTION] Maniobra completa. '
                        f'Paradas: {stops_completed}/{total_stops} | v={v:.3f} m/s')
                    state          = self.ST_COOLDOWN
                    cooldown_start = now

            # ----------------------------------------------------------
            # COOLDOWN: devolver throttle a pp_node
            # ----------------------------------------------------------
            elif state == self.ST_COOLDOWN:
                self._set_active(False)
                current_pwm = cruise_pwm

                elapsed_cd = now - cooldown_start
                sign_gone  = (not self._confirmed) or (dist < 0.0)

                if elapsed_cd >= self.COOLDOWN_MIN_S and sign_gone:
                    if stops_completed >= total_stops:
                        # MISION COMPLETA
                        self._set_done(True)
                        self.get_logger().info(
                            f'[ACTION] Mision completa: {stops_completed} paradas.')
                        goal_handle.succeed()
                        return self._make_result(
                            True, stops_completed,
                            f'Mision completada: {stops_completed} senales procesadas.')
                    else:
                        self.get_logger().info(
                            f'[ACTION] Cooldown OK -> SEARCHING '
                            f'(faltan {total_stops - stops_completed} paradas)')
                        state = self.ST_SEARCHING

            # -- Feedback y sleep de tick ------------------------------
            self._send_feedback(
                goal_handle, state, stops_completed, current_pwm,
                elapsed_wait=maneuver.wait_elapsed if maneuver else 0.0,
            )
            time.sleep(self.LOOP_DT)

        goal_handle.abort()
        return self._make_result(False, stops_completed, 'rclpy shutdown durante ejecucion')

    #  HELPER
    @staticmethod
    def _make_result(success: bool, stops: int, msg: str) -> StopRoutine.Result:
        r = StopRoutine.Result()
        r.success         = success
        r.stops_completed = stops
        r.message         = msg
        return r



def main(args=None):
    rclpy.init(args=args)
    node = StopRoutineActionServer()
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