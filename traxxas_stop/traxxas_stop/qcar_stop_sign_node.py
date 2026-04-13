import math
import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


class StopSignDetector(Node):
    def __init__(self):
        super().__init__('stop_sign_detector')

        self.bridge = CvBridge()

        self.declare_parameter('color_topic', '/qcar/decompressed/rgbd_color')
        self.declare_parameter('depth_topic', '/qcar/rgbd_depth')
        self.declare_parameter('min_area', 200.0)
        self.declare_parameter('debug', True)

        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.min_area = float(self.get_parameter('min_area').value)
        self.debug_enabled = bool(self.get_parameter('debug').value)

        # Guardamos el último frame de cada topic por separado
        self.latest_color = None
        self.latest_depth = None

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub_detected_raw = self.create_publisher(Bool, '/traxxas/stop_sign/detected_raw', 10)
        self.pub_distance = self.create_publisher(Float32, '/traxxas/stop_sign/distance_m', 10)
        self.pub_area = self.create_publisher(Float32, '/traxxas/stop_sign/area', 10)
        self.pub_debug = self.create_publisher(Image, '/traxxas/stop_sign/debug_image', 10)

        # Callbacks independientes — sin sincronizador
        self.create_subscription(Image, self.color_topic, self.color_callback, sensor_qos)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, sensor_qos)

        # Timer que procesa a 10 Hz con lo que haya disponible
        self.timer = self.create_timer(0.1, self.process)

        self.get_logger().info(f'Color topic: {self.color_topic}')
        self.get_logger().info(f'Depth topic: {self.depth_topic}')

    def color_callback(self, msg):
        self.latest_color = msg

    def depth_callback(self, msg):
        self.latest_depth = msg

    def process(self):
        if self.latest_color is None:
            self.get_logger().warn('Esperando imagen color...', throttle_duration_sec=5.0)
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_color, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo color: {e}')
            return

        depth = None
        if self.latest_depth is not None:
            try:
                depth = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='passthrough')
            except Exception as e:
                self.get_logger().error(f'Error convirtiendo depth: {e}')

        detected, debug, mask, best = self.detect_stop_sign(frame)

        raw_msg = Bool()
        raw_msg.data = detected
        self.pub_detected_raw.publish(raw_msg)

        area_msg = Float32()
        dist_msg = Float32()

        if detected and best is not None:
            area_msg.data = float(best['area'])

            if depth is not None:
                distance_m = self.get_depth_at_center(depth, best['cx'], best['cy'])
                dist_msg.data = distance_m if math.isfinite(distance_m) else -1.0
                dist_text = (f"dist={distance_m:.2f} m"
                             if math.isfinite(distance_m) else "dist=invalid")
            else:
                dist_msg.data = -1.0
                dist_text = "dist=no depth"

            cv2.putText(debug, dist_text,
                        (best['x'], best['y'] + best['h'] + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        else:
            area_msg.data = 0.0
            dist_msg.data = -1.0

        self.pub_area.publish(area_msg)
        self.pub_distance.publish(dist_msg)

        if self.debug_enabled:
            try:
                dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
                dbg_msg.header = self.latest_color.header
                self.pub_debug.publish(dbg_msg)
            except Exception as e:
                self.get_logger().error(f'Error publicando debug: {e}')

    def detect_stop_sign(self, frame):
        # --- igual que antes, sin cambios ---
        debug = frame.copy()
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 60, 40])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 60, 40])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            approx = cv2.approxPolyDP(cnt, 0.015 * perimeter, True)
            sides = len(approx)
            if sides < 5 or sides > 10:
                continue
            if not cv2.isContourConvex(approx):
                continue
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = w / float(h)
            if aspect_ratio < 0.75 or aspect_ratio > 1.25:
                continue
            rect_area = w * h
            fill_ratio = area / float(rect_area + 1e-6)
            if fill_ratio < 0.55:
                continue
            if area > best_area:
                cx = x + w // 2
                cy = y + h // 2
                best_area = area
                best = {
                    'approx': approx,
                    'x': x, 'y': y, 'w': w, 'h': h,
                    'cx': cx, 'cy': cy,
                    'area': float(area),
                    'sides': sides
                }

        if best is None:
            cv2.putText(debug, 'No stop sign detected', (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            return False, debug, mask, None

        cv2.drawContours(debug, [best['approx']], -1, (0, 255, 0), 3)
        cv2.rectangle(debug, (best['x'], best['y']),
                      (best['x'] + best['w'], best['y'] + best['h']), (255, 0, 0), 2)
        cv2.circle(debug, (best['cx'], best['cy']), 5, (0, 255, 255), -1)
        cv2.putText(debug,
                    f"STOP detected | area={int(best['area'])} | sides={best['sides']}",
                    (best['x'], max(30, best['y'] - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return True, debug, mask, best

    def get_depth_at_center(self, depth_img, cx, cy):
        h, w = depth_img.shape[:2]
        if not (0 <= cx < w and 0 <= cy < h):
            return float('nan')
        x0, x1 = max(0, cx - 2), min(w, cx + 3)
        y0, y1 = max(0, cy - 2), min(h, cy + 3)
        patch = depth_img[y0:y1, x0:x1]
        valid = patch[np.isfinite(patch)]
        valid = valid[valid > 0.0]
        return float(np.median(valid)) if valid.size > 0 else float('nan')


def main(args=None):
    rclpy.init(args=args)
    node = StopSignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()