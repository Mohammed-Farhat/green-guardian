#!/usr/bin/env python3
# ================================================================
# GREEN GUARDIAN — YOLO Detection Node
# Package:  robot_vision
# Node:     yolo_node  (runs on LAPTOP)
#
# Subscribes:
#   /camera/image_raw      (sensor_msgs/Image)
#
# Publishes:
#   /garbage_detection     (std_msgs/String)  "organic" | "non_organic" | "none"
#   /camera/yolo_annotated (sensor_msgs/Image)
#
# Temporal filter: uses TemporalConsistencyFilter (IoU-based spatial tracker).
#   A detection is confirmed only after appearing in the same location for
#   required_hits=5 consecutive frames (max_misses=2 allowed between frames).
#
# Acceptance bypass: bbox area < 1% of frame area → whitelisted immediately,
#   skips the temporal filter, drawn in purple.
#
# Model classes (from temporal_filter.py):
#   class_id 0 → "organic"
#   class_id 1 → "non_organic"
# ================================================================
import os

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from robot_vision.temporal_filter import Detection, TemporalConsistencyFilter

# BGR colours
_COLOR_CONFIRMED = (0, 200, 0)    # green  — passed temporal filter
_COLOR_WHITELIST = (180, 0, 180)  # purple — small-object bypass


class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo_node')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter(
            'model_path',
            os.path.expanduser('~/green-guardian/ros2_ws/src/best.pt'))
        self.declare_parameter('confidence_threshold', 0.7)

        model_path = os.path.expanduser(
            self.get_parameter('model_path').value)
        self.conf_thresh = float(
            self.get_parameter('confidence_threshold').value)

        # ── Load YOLO model ──────────────────────────────────────
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
        except Exception as exc:
            self.get_logger().error(f'Failed to load model: {exc}')
            raise

        self.get_logger().info(
            f'Model loaded. Classes: {self.model.names}  '
            f'Confidence threshold: {self.conf_thresh}')

        # ── Temporal filter ──────────────────────────────────────
        # required_hits=5 matches the "5 of last 10 frames" design intent;
        # IoU-based spatial tracking from temporal_filter.py is used instead
        # of a simple class-presence deque so that overlapping false positives
        # at different positions don't count toward the same tracked object.
        self._filter = TemporalConsistencyFilter(
            required_hits=5,
            iou_threshold=0.4,
            max_misses=2,
        )

        # ── CV bridge ────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── QoS ─────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Publishers ───────────────────────────────────────────
        self.det_pub = self.create_publisher(String, '/garbage_detection', 10)
        self.img_pub = self.create_publisher(
            Image, '/camera/yolo_annotated', sensor_qos)

        # ── Subscriber ───────────────────────────────────────────
        self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, sensor_qos)

        self.get_logger().info('YOLO node ready — waiting for /camera/image_raw')

    # ================================================================
    # IMAGE CALLBACK
    # ================================================================
    def _image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge conversion failed: {exc}')
            return

        img_h, img_w = frame.shape[:2]
        frame_area = img_h * img_w

        # ── Run YOLO inference ───────────────────────────────────
        try:
            results = self.model(frame, conf=self.conf_thresh, verbose=False)
        except Exception as exc:
            self.get_logger().warn(f'YOLO inference failed: {exc}')
            return

        # ── Separate raw detections into normal vs whitelisted ───
        # temporal_filter.Detection uses normalized [x_center, y_center, w, h]
        # which is what YOLO's xywhn output provides natively.
        normal_detections: list[Detection] = []   # go through temporal filter
        whitelisted: list[Detection] = []         # bypass temporal filter (small)

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                cls_id   = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                conf     = float(box.conf[0])

                # Normalized format: [x_center, y_center, width, height]
                xc_n, yc_n, w_n, h_n = (float(v) for v in box.xywhn[0])

                # Pixel area for the 1% threshold check
                x1 = int((xc_n - w_n / 2) * img_w)
                y1 = int((yc_n - h_n / 2) * img_h)
                x2 = int((xc_n + w_n / 2) * img_w)
                y2 = int((yc_n + h_n / 2) * img_h)
                pixel_area = max(0, x2 - x1) * max(0, y2 - y1)

                det = Detection(
                    class_id=cls_id,
                    class_name=cls_name,
                    confidence=conf,
                    x_center=xc_n,
                    y_center=yc_n,
                    width=w_n,
                    height=h_n,
                )

                if pixel_area < 0.01 * frame_area:
                    whitelisted.append(det)
                else:
                    normal_detections.append(det)

        # ── Run temporal filter on non-whitelisted detections ────
        confirmed = self._filter.update(normal_detections)

        # ── Determine best garbage label to publish ──────────────
        all_accepted = confirmed + whitelisted
        best_label = 'none'
        best_conf  = -1.0
        for det in all_accepted:
            # Class names in best.pt are exactly "organic" or "non_organic"
            if det.class_name in ('organic', 'non_organic') and det.confidence > best_conf:
                best_conf  = det.confidence
                best_label = det.class_name

        det_msg = String()
        det_msg.data = best_label
        self.det_pub.publish(det_msg)

        if best_label != 'none':
            self.get_logger().info(
                f'Detection published: {best_label}  (conf={best_conf:.2f}  '
                f'confirmed={len(confirmed)}  whitelisted={len(whitelisted)})')

        # ── Draw annotations ─────────────────────────────────────
        annotated = frame.copy()

        for det in confirmed:
            _draw_box(annotated, det, img_w, img_h,
                      _COLOR_CONFIRMED, whitelisted=False)

        for det in whitelisted:
            _draw_box(annotated, det, img_w, img_h,
                      _COLOR_WHITELIST, whitelisted=True)

        # ── Publish annotated image ───────────────────────────────
        try:
            ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            ann_msg.header = msg.header
            self.img_pub.publish(ann_msg)
        except Exception as exc:
            self.get_logger().warn(f'Annotated image publish failed: {exc}')


# ================================================================
# DRAWING HELPER
# ================================================================
def _draw_box(frame, det: Detection, img_w: int, img_h: int,
              color: tuple, whitelisted: bool):
    x1 = int((det.x_center - det.width  / 2) * img_w)
    y1 = int((det.y_center - det.height / 2) * img_h)
    x2 = int((det.x_center + det.width  / 2) * img_w)
    y2 = int((det.y_center + det.height / 2) * img_h)

    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

    prefix = '[S] ' if whitelisted else ''
    label  = f'{prefix}{det.class_name} {det.confidence:.2f}'
    cv2.putText(frame, label, (x1, max(y1 - 6, 0)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)


# ================================================================
# ENTRY POINT
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
