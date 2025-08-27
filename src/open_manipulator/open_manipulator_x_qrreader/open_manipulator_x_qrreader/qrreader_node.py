#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ultralytics import YOLO
import cv2 as cv
import numpy as np
from pyzbar import pyzbar
import time

from .sort import Sort


class QrReaderNode(Node):
    """
    YOLO + QR 디코딩 → 허용 지역이면 /qr_info 퍼블리시
    (엣지 트리거: 보이는 동안 1번만 퍼블리시, 사라졌다가 다시 보이면 다시 퍼블리시)
    """

    # ---------- 고정 설정 ----------
    MODEL_FILENAME   = "/home/pyo/Downloads/best7.pt"
    CAMERA_INDEX     = 0
    CAMERA_FALLBACK  = 1
    CAMERA_WIDTH     = 640
    CAMERA_HEIGHT    = 640

    YOLO_CONF_THR    = 0.5
    YOLO_IOU_THR     = 0.4
    TRACK_CONF_THR   = 0.6

    ALLOWED_ZONES    = {'서울', '인천', '부산'}
    VISUALIZE        = True

    def __init__(self):
        super().__init__('qr_reader_node')

        # --- ROS 파라미터(필요시 런치에서 변경)
        self.declare_parameter('qr_vanish_timeout_sec', 1.0)
        self.vanish_timeout = float(self.get_parameter('qr_vanish_timeout_sec').value)

        # 퍼블리셔
        self.publisher_ = self.create_publisher(String, '/qr_info', 10)

        # YOLO 모델 로드
        try:
            self.model = YOLO(self.MODEL_FILENAME)
            self.get_logger().info(f"YOLO 모델 로드 완료: {self.MODEL_FILENAME}")
        except Exception as e:
            self.get_logger().error(f"YOLO 모델 로드 실패: {e}")
            raise

        self.tracker = Sort()

        # 카메라 초기화
        self.cap = cv.VideoCapture(self.CAMERA_INDEX, cv.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().warn(f"카메라 {self.CAMERA_INDEX} 열기 실패 → {self.CAMERA_FALLBACK} 시도")
            self.cap.release()
            self.cap = cv.VideoCapture(self.CAMERA_FALLBACK, cv.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error(
                f"카메라 열기 실패 (index={self.CAMERA_INDEX} / fallback={self.CAMERA_FALLBACK})"
            )
            raise RuntimeError("Camera open failed")

        self.cap.set(cv.CAP_PROP_FRAME_WIDTH,  self.CAMERA_WIDTH)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.CAMERA_HEIGHT)

        # 엣지 트리거 상태
        self.active_qrs = set()  # 현재 보이는 것으로 간주되는 QR 문자열 집합
        self.last_seen  = {}     # { qr_string: 마지막으로 본 epoch time(sec) }

        self.get_logger().info("QR Reader Node 시작! (허용 지역이면 /qr_info 퍼블리시)")
        self.get_logger().info("지역 분류 대상: " + ", ".join(sorted(self.ALLOWED_ZONES)))
        self.get_logger().info(
            f"사용 중인 카메라 index: {self.CAMERA_INDEX if self.cap.isOpened() else self.CAMERA_FALLBACK}"
        )
        self.get_logger().info(f"QR vanish timeout: {self.vanish_timeout:.2f}s")

        # 주기 실행
        self.timer = self.create_timer(0.1, self.detect_loop)

    # === 메인 루프 ===
    def detect_loop(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warning("카메라 프레임 읽기 실패")
            return

        annotated = frame.copy()

        # YOLO 감지
        try:
            results = self.model.predict(
                source=frame, imgsz=640,
                conf=self.YOLO_CONF_THR, iou=self.YOLO_IOU_THR,
                verbose=False, show=False
            )
        except Exception as e:
            self.get_logger().warning(f"YOLO 추론 예외: {e}")
            return

        # 추적 시각화를 원하면 유지 (로직에는 영향 없음)
        dets = []
        if len(results) > 0 and hasattr(results[0], "boxes"):
            for box in results[0].boxes:
                conf = float(box.conf[0].item())
                if conf >= self.TRACK_CONF_THR:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    dets.append([x1, y1, x2, y2, conf])

        dets = np.array(dets) if dets else np.empty((0, 5))
        tracks = self.tracker.update(dets)
        for d in tracks:
            x1, y1, x2, y2, tid = map(int, d[:5])
            if self.VISUALIZE:
                cv.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv.putText(annotated, f"BOX ID:{tid}", (x1, max(0, y1 - 10)),
                           cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # QR 디코드 → 허용 지역이면 /qr_info 퍼블리시 (엣지 트리거)
        decoded = pyzbar.decode(frame)
        now = time.time()

        seen_now = set()
        for qr in decoded:
            data = qr.data.decode('utf-8', errors='ignore').strip()
            x, y, w, h = qr.rect
            if self.VISUALIZE:
                cv.rectangle(annotated, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv.putText(annotated, data, (x, max(0, y - 10)),
                           cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

            if not data:
                continue

            seen_now.add(data)
            self.last_seen[data] = now

            if (not self.ALLOWED_ZONES) or (data in self.ALLOWED_ZONES):
                # 처음 보인 경우에만 퍼블리시 (엣지 트리거)
                if data not in self.active_qrs:
                    msg = String()
                    msg.data = data
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"/qr_info 퍼블리시(신규 감지): {data}")
                    self.active_qrs.add(data)
            else:
                self.get_logger().warn(f"허용되지 않은 QR값: {data}")

        # vanish_timeout 동안 안 보인 QR은 active에서 제거
        to_remove = []
        for q in list(self.active_qrs):
            last = self.last_seen.get(q, 0.0)
            if now - last > self.vanish_timeout:
                to_remove.append(q)
        for q in to_remove:
            self.active_qrs.remove(q)

        if self.VISUALIZE:
            try:
                cv.imshow("BOXES & Tracking & QR", annotated)
                if cv.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info("종료 요청(q)")
                    rclpy.shutdown()
            except cv.error:
                pass

    def destroy_node(self):
        try:
            if self.cap:
                self.cap.release()
        finally:
            try:
                cv.destroyAllWindows()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QrReaderNode()
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
