#obsoleto

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32


class StopMissionManager(Node):
    def __init__(self):
        super().__init__('stop_mission_manager')

        self.declare_parameter('prepare_distance', 0.6)
        self.declare_parameter('brake_distance', 0.5)
        self.declare_parameter('stop_distance', 0.4)
        self.declare_parameter('wait_seconds', 5.0)
        self.declare_parameter('cruise_speed', 1.0)
        self.declare_parameter('approach_speed', 0.5)
        self.declare_parameter('brake_speed', 0.2)

        self.prepare_distance = float(self.get_parameter('prepare_distance').value)
        self.brake_distance = float(self.get_parameter('brake_distance').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.wait_seconds = float(self.get_parameter('wait_seconds').value)
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.approach_speed = float(self.get_parameter('approach_speed').value)
        self.brake_speed = float(self.get_parameter('brake_speed').value)

        self.confirmed = False
        self.distance_m = -1.0

        self.state = 'SEARCHING'
        self.stop_done = False
        self.stop_start_time = None

        self.create_subscription(Bool, '/traxxas/stop_sign/confirmed', self.confirmed_callback, 10)
        self.create_subscription(Float32, '/traxxas/stop_sign/distance_m', self.distance_callback, 10)

        self.active_pub = self.create_publisher(Bool, '/traxxas/stop_sign/active', 10)
        self.speed_override_pub = self.create_publisher(Float32, '/traxxas/stop_sign/speed_override', 10)
        self.done_pub = self.create_publisher(Bool, '/traxxas/stop_sign/done', 10)

        self.timer = self.create_timer(0.05, self.update_state_machine)

        self.get_logger().info('StopMissionManager iniciado')

    def confirmed_callback(self, msg):
        self.confirmed = bool(msg.data)

    def distance_callback(self, msg):
        self.distance_m = float(msg.data)

    def publish_outputs(self, active, speed, done):
        active_msg = Bool()
        active_msg.data = active
        self.active_pub.publish(active_msg)

        speed_msg = Float32()
        speed_msg.data = float(speed)
        self.speed_override_pub.publish(speed_msg)

        done_msg = Bool()
        done_msg.data = done
        self.done_pub.publish(done_msg)

    def update_state_machine(self):
        if self.stop_done and not self.confirmed:
            self.stop_done = False
            self.state = 'SEARCHING'
            self.get_logger().info('Reset, listo para siguiente señal de alto')

        if self.stop_done:
            self.publish_outputs(False, self.cruise_speed, True)
            return

        if self.state == 'SEARCHING':
            if self.confirmed and self.distance_m > 0.0 and self.distance_m <= self.prepare_distance:
                self.state = 'APPROACHING'

            self.publish_outputs(False, self.cruise_speed, False)
            return

        if self.state == 'APPROACHING':
            if self.distance_m > 0.0:
                if self.distance_m <= self.stop_distance:
                    self.state = 'STOPPED_WAITING'
                    self.stop_start_time = time.time()
                    self.publish_outputs(True, 0.0, False)
                    return

                if self.distance_m <= self.brake_distance:
                    self.publish_outputs(True, self.brake_speed, False)
                    return

            self.publish_outputs(True, self.approach_speed, False)
            return

        if self.state == 'STOPPED_WAITING':
            elapsed = time.time() - self.stop_start_time if self.stop_start_time is not None else 0.0

            if elapsed >= self.wait_seconds:
                self.state = 'RESUMING'
                self.publish_outputs(True, self.approach_speed, False)
                return

            self.publish_outputs(True, 0.0, False)
            return

        if self.state == 'RESUMING':
            self.stop_done = True
            self.state = 'DONE'
            self.publish_outputs(False, self.cruise_speed, True)
            return


def main(args=None):
    rclpy.init(args=args)
    node = StopMissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()