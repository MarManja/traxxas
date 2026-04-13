#!/usr/bin/env python3
"""
=======================================================================
 Pure Pursuit Controller for Physical QCar using ROS2
 Author: Marmanja / Iván Valdez del Toro
 Adaptado para integración con QCarStopRoutineActionServer
-----------------------------------------------------------------------
 CAMBIOS vs versión anterior
 ────────────────────────────
   - La lógica de parada por señal de alto ya NO vive aquí.
     La gestiona el action server (qcar_stop_routine_action_server).
   - Este nodo se suscribe a /traxxas/stop_sign/active (Bool):
       False → publica normalmente en /qcar/user_command
       True  → publica throttle=0.0 (cede control al action server)
               pero sigue calculando el steering
   - v_ref default = 0.04 (velocidad crucero QCar)
   - max_steer_cmd = 0.3 (límite físico QCar)
=======================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, Float32

import math
import csv
from pathlib import Path
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt


class PPState:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.xr = x; self.yr = y; self.theta = yaw; self.v = v

    def calc_distance(self, px, py):
        return math.hypot(self.xr - px, self.yr - py)


class TargetCourse:
    def __init__(self, path_points, k_gain, base_lookahead):
        self.cx = [p[0] for p in path_points]
        self.cy = [p[1] for p in path_points]
        self.k = k_gain; self.Lfc = base_lookahead
        self.old_nearest_point_index = None

    def search_target_index(self, state: PPState):
        if self.old_nearest_point_index is None:
            dists = [math.hypot(state.xr - cx_i, state.yr - cy_i)
                     for cx_i, cy_i in zip(self.cx, self.cy)]
            ind = int(np.argmin(dists))
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this = state.calc_distance(self.cx[ind], self.cy[ind])
            while ind + 1 < len(self.cx):
                distance_next = state.calc_distance(self.cx[ind+1], self.cy[ind+1])
                if distance_this < distance_next:
                    break
                ind += 1; distance_this = distance_next
            self.old_nearest_point_index = ind

        Lf = self.k * max(state.v, 0.0) + self.Lfc
        target_ind = ind
        while target_ind < len(self.cx) - 1:
            if state.calc_distance(self.cx[target_ind], self.cy[target_ind]) >= Lf:
                break
            target_ind += 1
        return target_ind, Lf


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.declare_parameter('path_csv', '')
        self.declare_parameter('circle_radius', 0.8)
        self.declare_parameter('circle_points', 300)
        self.declare_parameter('lookahead', 0.1)
        self.declare_parameter('k_gain', 0.8)
        self.declare_parameter('v_ref', 0.06)
        self.declare_parameter('wheelbase', 0.256)

        path_csv       = self.get_parameter('path_csv').get_parameter_value().string_value
        self.lookahead = self.get_parameter('lookahead').get_parameter_value().double_value
        self.v_ref     = self.get_parameter('v_ref').get_parameter_value().double_value
        self.k_gain    = self.get_parameter('k_gain').get_parameter_value().double_value
        self.L         = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.max_steer_cmd = 0.3

        self.stop_active       = False
        self.current_pose      = None
        self.state             = PPState()
        self.target_ind        = 0
        self.last_objective    = None
        self.stop_active       = False
        self.stop_throttle_cmd = 0.0

        self.generated_trajectory = []
        self.log_t = []; self.log_x = []; self.log_y = []
        self.log_yaw = []; self.log_v = []
        self.start_time = self.get_clock().now()

        if path_csv.strip():
            self.path = self.load_path(path_csv)
        else:
            self.path = self.generate_circle_path()

        self.target_course = TargetCourse(self.path, self.k_gain, self.lookahead) if self.path else None

        self.create_subscription(Vector3Stamped, '/qcar/pose', 
                                 self.pose_callback, 10)

        self.create_subscription(Bool, '/traxxas/stop_sign/active',
                                 self.stop_active_callback, 10)
        
        self.create_subscription(Float32, '/qcar/stop_throttle',
                                 self.stop_throttle_callback, 10)

        self.pub = self.create_publisher(Vector3Stamped, '/qcar/user_command', 10)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("=" * 55)
        self.get_logger().info(" PurePursuitNode QCar iniciado")
        self.get_logger().info(f"  v_ref={self.v_ref} | L={self.L} m | steer_max=±{self.max_steer_cmd}")
        self.get_logger().info("  Cede throttle cuando /traxxas/stop_sign/active=True")
        self.get_logger().info("=" * 55)

    def pose_callback(self, msg: Vector3Stamped):
        self.current_pose = msg

    def stop_active_callback(self, msg: Bool):
        was = self.stop_active
        self.stop_active = bool(msg.data)
        if self.stop_active != was:
            if self.stop_active:
                self.get_logger().warn('Action server activo.')
            else:
                self.get_logger().info('Action server liberó — reanudando PP.')

    def stop_throttle_callback(self, msg: Float32):
        self.stop_throttle_cmd = float(msg.data) 

    def control_loop(self):
        now = self.get_clock().now()
        if self.current_pose is None or not self.path or self.target_course is None:
            return

        x_r = self.current_pose.vector.x
        y_r = self.current_pose.vector.y
        yaw = self.current_pose.vector.z

        self.state.xr = x_r; self.state.yr = y_r
        self.state.theta = yaw; self.state.v = abs(self.v_ref)

        delta     = self.compute_pure_pursuit_delta()
        steer_cmd = max(-self.max_steer_cmd, min(self.max_steer_cmd, delta))

        # Si el action server tiene el control, throttle cambia a lo del action
        # El watchdog no dispara porque el action server también publica
        throttle = self.stop_throttle_cmd if self.stop_active else self.v_ref

        cmd = Vector3Stamped()
        cmd.header.stamp    = now.to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.vector.x = float(throttle)
        cmd.vector.y = float(-steer_cmd)   # QCar: -0.3=der, +0.3=izq
        cmd.vector.z = 0.0
        self.pub.publish(cmd)

        if not self.stop_active:
            self.generated_trajectory.append((x_r, y_r))
            t = (now.nanoseconds - self.start_time.nanoseconds) * 1e-9
            self.log_t.append(t); self.log_x.append(x_r); self.log_y.append(y_r)
            self.log_yaw.append(yaw); self.log_v.append(self.v_ref)

    def compute_pure_pursuit_delta(self):
        ind, Lf = self.target_course.search_target_index(self.state)
        if self.target_ind >= ind:
            ind = self.target_ind
        self.target_ind = ind

        tx = self.target_course.cx[ind]
        ty = self.target_course.cy[ind]

        if self.last_objective is None or ind != self.last_objective:
            self.last_objective = ind
            self.get_logger().info(
                f"Objetivo={ind} | ({tx:.3f},{ty:.3f}) | "
                f"dist={self.state.calc_distance(tx,ty):.3f} m | Lf={Lf:.3f}"
            )

        alpha = math.atan2(ty - self.state.yr, tx - self.state.xr) - self.state.theta
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        return math.atan2(2.0 * self.L * math.sin(alpha), max(Lf, 1e-3))

    def load_path(self, csv_file):
        p = Path(csv_file)
        if not p.exists():
            self.get_logger().error(f'CSV no encontrado: {csv_file}'); return []
        points = []
        with p.open('r') as f:
            for row in csv.DictReader(f):
                try: points.append((float(row['x']), float(row['y'])))
                except (KeyError, ValueError): continue
        self.get_logger().info(f'Path: {len(points)} puntos')
        return points

    def generate_circle_path(self):
        R = self.get_parameter('circle_radius').get_parameter_value().double_value
        N = max(int(self.get_parameter('circle_points').get_parameter_value().integer_value), 3)
        cx, cy = 0.0, R
        pts = [(cx + R*math.cos(2*math.pi*i/N), cy + R*math.sin(2*math.pi*i/N))
               for i in range(N)]
        self.get_logger().info(f'Trayectoria circular: R={R:.3f} m, N={N}')
        return pts

    # def plot_results(self):
    #     if not self.path or not self.generated_trajectory:
    #         print("No hay datos para graficar."); return

    #     exp_x = [p[0] for p in self.path]; exp_y = [p[1] for p in self.path]
    #     gen_x = [p[0] for p in self.generated_trajectory]
    #     gen_y = [p[1] for p in self.generated_trajectory]

    #     errors = []
    #     for gp in self.generated_trajectory:
    #         d = min(math.hypot(gp[0]-rp[0], gp[1]-rp[1]) for rp in self.path)
    #         errors.append(d)
    #     avg_e = float(np.mean(errors)) if errors else 0.0
    #     max_e = float(np.max(errors))  if errors else 0.0
    #     print(f"\nError promedio: {avg_e:.3f} m | máximo: {max_e:.3f} m")

    #     out_dir = (Path.home() / 'Workspaces' / 'traxxas_ws'
    #                / 'resultados' / 'pure_pursuit_qcar')
    #     out_dir.mkdir(parents=True, exist_ok=True)
    #     ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
    #     tag = f"L{self.lookahead}_V{self.v_ref}"

    #     if self.log_t:
    #         fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    #         axes[0,0].plot(exp_x, exp_y, '-r', label='Ref', lw=2)
    #         axes[0,0].plot(gen_x, gen_y, '-b', label='Real', lw=1.5)
    #         axes[0,0].set_title('Trayectorias'); axes[0,0].legend()
    #         axes[0,0].grid(True); axes[0,0].axis('equal')

    #         if errors:
    #             axes[0,1].plot(self.log_t[:len(errors)], errors, '-g')
    #             axes[0,1].axhline(y=avg_e, linestyle='--', label=f'Prom:{avg_e:.3f}m')
    #             axes[0,1].set_title('Error vs tiempo')
    #             axes[0,1].legend(); axes[0,1].grid(True)

    #         axes[1,0].plot(self.log_t, [v*3.6 for v in self.log_v], '-m')
    #         axes[1,0].set_title('Velocidad cmd [km/h]'); axes[1,0].grid(True)

    #         axes[1,1].plot(self.log_t, [math.degrees(th) for th in self.log_yaw], '-c')
    #         axes[1,1].set_title('Orientación [°]'); axes[1,1].grid(True)

    #         plt.tight_layout()
    #         fp = out_dir / f"pp_qcar_{ts}_{tag}.png"
    #         fig.savefig(fp, dpi=300, bbox_inches='tight'); plt.close(fig)
    #         print(f"Gráfica: {fp}")


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_results()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()