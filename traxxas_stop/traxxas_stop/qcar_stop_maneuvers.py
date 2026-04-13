#!/usr/bin/env python3
"""
=======================================================================
 QCarStopManeuvers  -  traxxas_stop
 Adaptado para QCar por: [tu nombre]
 Original (Traxxas/PWM): Marmanja
-----------------------------------------------------------------------
 Clases de maniobra de parada CON retroalimentación del encoder QCar.

 DIFERENCIAS vs versión Traxxas
 ───────────────────────────────
   - En lugar de PWM (int) usa throttle_cmd (float) en rango QCar:
       0.0   = detenido  (equivale a brake_pwm=2457)
       0.04  = crucero   (equivale a cruise_pwm=2570)
       0.06  = máximo    (equivale a max_pwm=2600)
   - La velocidad del encoder llega de /qcar/velocity (Vector3Stamped,
     vector.x en m/s) — el action server extrae el float y lo pasa.
   - La lógica de BrakeRamp / StopWait / ResumeRamp / StopManeuver
     es idéntica a la versión Traxxas, solo cambia el tipo de retorno.

 Clases
 ──────
   BrakeRamp    – rampa de frenado + espera |v| < stopped_threshold
   StopWait     – quieto wait_s segundos
   ResumeRamp   – rampa de arranque + espera v > stable_threshold
   StopManeuver – agrupa las tres con FSM BRAKING→STOPPED→RESUMING→DONE
=======================================================================
"""
import time


class BrakeRamp:
    """
    Reduce throttle de start_cmd a 0.0 en duration_s segundos,
    luego espera que el encoder confirme |v| < stopped_threshold.

    Parámetros
    ----------
    start_cmd         : throttle al iniciar (ej. 0.04)
    duration_s        : segundos de rampa
    stopped_threshold : velocidad [m/s] bajo la cual = detenido (0.03)
    encoder_timeout_s : máximo extra de espera si encoder no confirma (3.0)
    """

    def __init__(
        self,
        start_cmd        : float,
        duration_s       : float,
        stopped_threshold: float = 0.03,
        encoder_timeout_s: float = 3.0,
    ):
        self.start_cmd         = float(start_cmd)
        self.duration_s        = max(float(duration_s), 0.01)
        self.stopped_threshold = float(stopped_threshold)
        self.encoder_timeout_s = float(encoder_timeout_s)
        self._t0              = 0.0
        self._ramp_done       = False
        self._encoder_done    = False
        self._enc_wait_start  = 0.0

    def start(self):
        self._t0            = time.time()
        self._ramp_done     = False
        self._encoder_done  = False

    @property
    def done(self) -> bool:
        return self._ramp_done and self._encoder_done

    @property
    def elapsed(self) -> float:
        return time.time() - self._t0

    def tick(self, v_mps: float) -> float:
        """
        Retorna throttle_cmd (float) a publicar.
        v_mps: velocidad del encoder en m/s.
        """
        t = self.elapsed / self.duration_s

        if not self._ramp_done:
            if t >= 1.0:
                self._ramp_done      = True
                self._enc_wait_start = time.time()
            cmd = self.start_cmd * (1.0 - min(t, 1.0))   # rampa de bajada
            return max(0.0, cmd)

        # Rampa terminada — esperar que el encoder confirme parada
        if not self._encoder_done:
            if abs(v_mps) < self.stopped_threshold:
                self._encoder_done = True
            elif (time.time() - self._enc_wait_start) > self.encoder_timeout_s:
                self._encoder_done = True   # timeout de seguridad
        return 0.0


class StopWait:
    """
    Mantiene throttle=0.0 durante wait_s segundos.

    Parámetros
    ----------
    wait_s : segundos de espera
    """

    def __init__(self, wait_s: float):
        self.wait_s = max(float(wait_s), 0.0)
        self._t0    = 0.0
        self._done  = False

    def start(self):
        self._t0   = time.time()
        self._done = False

    @property
    def done(self) -> bool:
        return self._done

    @property
    def elapsed(self) -> float:
        return time.time() - self._t0

    def tick(self, v_mps: float = 0.0) -> float:
        """v_mps aceptado para firma uniforme pero no se usa."""
        if time.time() - self._t0 >= self.wait_s:
            self._done = True
        return 0.0


class ResumeRamp:
    """
    Sube throttle de 0.0 → cruise_cmd en duration_s segundos,
    luego espera que el encoder confirme v > stable_threshold.

    Parámetros
    ----------
    cruise_cmd        : throttle objetivo de crucero (ej. 0.04)
    duration_s        : segundos de rampa de arranque
    stable_threshold  : velocidad [m/s] que = "en marcha" (0.02)
    encoder_timeout_s : máximo de espera si encoder no confirma (4.0)
    """

    def __init__(
        self,
        cruise_cmd       : float,
        duration_s       : float,
        stable_threshold : float = 0.02,
        encoder_timeout_s: float = 4.0,
    ):
        self.cruise_cmd       = float(cruise_cmd)
        self.duration_s       = max(float(duration_s), 0.01)
        self.stable_threshold = float(stable_threshold)
        self.encoder_timeout_s = float(encoder_timeout_s)
        self._t0              = 0.0
        self._ramp_done       = False
        self._encoder_done    = False
        self._enc_wait_start  = 0.0

    def start(self):
        self._t0           = time.time()
        self._ramp_done    = False
        self._encoder_done = False

    @property
    def done(self) -> bool:
        return self._ramp_done and self._encoder_done

    @property
    def elapsed(self) -> float:
        return time.time() - self._t0

    def tick(self, v_mps: float) -> float:
        """Retorna throttle_cmd a publicar."""
        t = self.elapsed / self.duration_s

        if not self._ramp_done:
            if t >= 1.0:
                self._ramp_done      = True
                self._enc_wait_start = time.time()
            cmd = self.cruise_cmd * min(t, 1.0)   # rampa de subida
            return min(self.cruise_cmd, max(0.0, cmd))

        # Rampa terminada — esperar encoder
        if not self._encoder_done:
            if abs(v_mps) >= self.stable_threshold:
                self._encoder_done = True
            elif (time.time() - self._enc_wait_start) > self.encoder_timeout_s:
                self._encoder_done = True
        return self.cruise_cmd


class StopManeuver:
    """
    Agrupa BrakeRamp + StopWait + ResumeRamp en una maniobra completa.
    FSM interna: BRAKING → STOPPED → RESUMING → DONE

    Parámetros
    ----------
    cruise_cmd        : throttle de crucero del QCar (ej. 0.04)
    start_cmd         : throttle al iniciar el frenado (normalmente = cruise_cmd)
    brake_dur         : segundos de rampa de frenado
    wait_s            : segundos detenido
    resume_dur        : segundos de rampa de arranque
    stopped_threshold : |v| [m/s] para confirmar detención (encoder)
    stable_threshold  : v [m/s] para confirmar rearranque (encoder)

    Uso
    ---
        m = StopManeuver(cruise_cmd=0.04, brake_dur=2.0, wait_s=5.0, resume_dur=1.5)
        m.start()
        while not m.done:
            throttle_cmd, phase = m.tick(v_mps)
            publish_throttle(throttle_cmd)
            send_feedback(phase, m.wait_elapsed)
    """

    BRAKING  = 'BRAKING'
    STOPPED  = 'STOPPED'
    RESUMING = 'RESUMING'
    DONE     = 'DONE'

    def __init__(
        self,
        cruise_cmd       : float = 0.04,
        start_cmd        : float = None,     # None = usar cruise_cmd
        brake_dur        : float = 2.0,
        wait_s           : float = 5.0,
        resume_dur       : float = 1.5,
        stopped_threshold: float = 0.03,
        stable_threshold : float = 0.02,
    ):
        if start_cmd is None:
            start_cmd = cruise_cmd

        self._brake  = BrakeRamp(
            start_cmd, brake_dur,
            stopped_threshold=stopped_threshold,
        )
        self._wait   = StopWait(wait_s)
        self._resume = ResumeRamp(
            cruise_cmd, resume_dur,
            stable_threshold=stable_threshold,
        )
        self._cruise_cmd = float(cruise_cmd)
        self._phase      = self.BRAKING
        self._done       = False

    def start(self):
        """Inicia la maniobra completa. Llamar una sola vez."""
        self._phase = self.BRAKING
        self._done  = False
        self._brake.start()

    @property
    def done(self) -> bool:
        return self._done

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def wait_elapsed(self) -> float:
        """Segundos en fase STOPPED. 0 si aún no empezó."""
        if self._phase in (self.STOPPED, self.RESUMING, self.DONE):
            return self._wait.elapsed
        return 0.0

    def tick(self, v_mps: float) -> tuple:
        """
        Avanza un tick con retroalimentación del encoder.
        v_mps : velocidad lineal del encoder [m/s] (de /qcar/velocity vector.x)
        Retorna (throttle_cmd: float, phase: str).
        """
        if self._phase == self.BRAKING:
            cmd = self._brake.tick(v_mps)
            if self._brake.done:
                self._phase = self.STOPPED
                self._wait.start()
            return cmd, self._phase

        if self._phase == self.STOPPED:
            cmd = self._wait.tick(v_mps)
            if self._wait.done:
                self._phase = self.RESUMING
                self._resume.start()
            return cmd, self._phase

        if self._phase == self.RESUMING:
            cmd = self._resume.tick(v_mps)
            if self._resume.done:
                self._phase = self.DONE
                self._done  = True
            return cmd, self._phase

        return self._cruise_cmd, self.DONE