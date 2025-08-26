#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
from typing import Optional, List
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None  # pyserial 미설치 대비


class UartMultiReceiver(Node):
    """
    - UART에서 한 줄씩 읽어 String/Float32 퍼블리시
    - /vacuum_cmd(True) 수신 → MCU로 '3' 전송 (집기/시작)
    - /manipulator_done == "vacuum off" 수신 → MCU로 '3' 전송 (놓기/해제)
    - /vacuum_cmd(False)는 전송 안 함(OFF는 "vacuum off" 타이밍에서 처리)
    - MCU 개행: none|lf|crlf (기본 none; 1바이트만 보냄)
    - 포트 오류 자동 재연결, 송신 디바운스
    """

    def __init__(self):
        super().__init__('uart_multi_receiver')

        # --- ROS Parameters (기본값을 코드에 고정적으로 둠)
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('reconnect_sec', 3.0)
        self.declare_parameter('publish_empty_lines', False)

        # 퍼블리시 관련
        self.declare_parameter('topic_raw', 'nucleo_uart')
        self.declare_parameter('csv_hint', True)
        self.declare_parameter('enable_float', True)
        self.declare_parameter('topic_float', '/ultrasonic_distance')

        # 완료/상태 토픽
        self.declare_parameter('done_topic', '/manipulator_done')

        # Vacuum 제어 토픽
        self.declare_parameter('vacuum_cmd_topic', '/vacuum_cmd')

        # MCU 개행 요구: none | lf | crlf  (기본 none)
        self.declare_parameter('tx_newline_mode', 'none')
        self.declare_parameter('retry_on_fail', True)
        self.declare_parameter('debounce_ms', 120)

        # --- Load params
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = int(self.get_parameter('baudrate').get_parameter_value().integer_value)
        self.reconnect_sec = float(self.get_parameter('reconnect_sec').get_parameter_value().double_value)
        self.publish_empty = self.get_parameter('publish_empty_lines').get_parameter_value().bool_value

        self.topic_raw = self.get_parameter('topic_raw').get_parameter_value().string_value
        self.csv_hint = self.get_parameter('csv_hint').get_parameter_value().bool_value
        self.enable_float = self.get_parameter('enable_float').get_parameter_value().bool_value
        self.topic_float = self.get_parameter('topic_float').get_parameter_value().string_value

        self.done_topic = self.get_parameter('done_topic').get_parameter_value().string_value
        self.vacuum_cmd_topic = self.get_parameter('vacuum_cmd_topic').get_parameter_value().string_value

        self.tx_newline_mode = self.get_parameter('tx_newline_mode').get_parameter_value().string_value.lower()
        self.retry_on_fail = self.get_parameter('retry_on_fail').get_parameter_value().bool_value
        self.debounce_ms = int(self.get_parameter('debounce_ms').get_parameter_value().integer_value or 120)

        if serial is None:
            self.get_logger().fatal('pyserial이 없습니다. `pip install pyserial` 후 실행하세요.')
            raise SystemExit(1)

        # --- Publishers
        self.pub_raw = self.create_publisher(String, self.topic_raw, 10)
        self.pub_float = (self.create_publisher(Float32, self.topic_float, 10) if self.enable_float else None)

        # --- Subscribers
        self.sub_done = self.create_subscription(String, self.done_topic, self._on_done_msg, 10)
        self.sub_vac  = self.create_subscription(Bool,   self.vacuum_cmd_topic, self._on_vacuum_cmd, 10)

        # --- Serial thread
        self._ser: Optional['serial.Serial'] = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self._th = threading.Thread(target=self._read_loop, daemon=True)
        self._th.start()

        # Debounce
        self._last_tx_ns = 0

        self.get_logger().info(
            f'UART Multi Receiver started: port={self.port}, baud={self.baud}, '
            f'raw_topic={self.topic_raw}, float_topic={self.topic_float if self.enable_float else "disabled"}, '
            f'done_topic={self.done_topic}, vacuum_cmd_topic={self.vacuum_cmd_topic}, '
            f'newline_mode={self.tx_newline_mode}'
        )

    # ---- Lifecycle
    def destroy_node(self):
        self._stop.set()
        try:
            if self._th.is_alive():
                self._th.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        super().destroy_node()

    # ---- Serial helpers
    def _open_serial(self) -> Optional['serial.Serial']:
        while not self._stop.is_set():
            try:
                ser = serial.Serial(self.port, self.baud, timeout=1.0, write_timeout=1.0)
                self.get_logger().info(f'Connected to {self.port} @ {self.baud}')
                return ser
            except Exception as e:
                self.get_logger().warn(f'포트 열기 실패: {e}. {self.reconnect_sec:.1f}s 후 재시도...')
                time.sleep(self.reconnect_sec)
        return None

    def _compose_payload(self, base: str) -> bytes:
        mode = self.tx_newline_mode
        if mode == 'lf':
            base += '\n'
        elif mode == 'crlf':
            base += '\r\n'
        return base.encode('utf-8')

    def _debounced(self) -> bool:
        now_ns = time.time_ns()
        if self._last_tx_ns != 0 and (now_ns - self._last_tx_ns) < self.debounce_ms * 1_000_000:
            return True
        self._last_tx_ns = now_ns
        return False

    def _send_byte(self, ch: str, label: str):
        if self._debounced():
            self.get_logger().debug("debounced uart send")
            return
        payload = self._compose_payload(ch)
        try:
            with self._write_lock:
                if self._ser is None or not self._ser.is_open:
                    self._ser = self._open_serial()
                    if self._ser is None:
                        self.get_logger().error("UART 연결 실패")
                        return
                self._ser.reset_output_buffer()
                n = self._ser.write(payload)
                self._ser.flush()
                if n != len(payload):
                    self.get_logger().error(f"UART partial write: {n}/{len(payload)}")
                self.get_logger().info(f"UART TX ({label}) -> {repr(payload)}")
        except Exception as e:
            self.get_logger().error(f"UART TX 실패: {e}")
            if self.retry_on_fail:
                time.sleep(0.05)
                try:
                    with self._write_lock:
                        if self._ser is None or not self._ser.is_open:
                            self._ser = self._open_serial()
                            if self._ser is None:
                                self.get_logger().error("재연결 실패(재시도 불가)")
                                return
                        n = self._ser.write(payload)
                        self._ser.flush()
                        if n == len(payload):
                            self.get_logger().info("UART TX 재시도 성공")
                        else:
                            self.get_logger().error(f"UART TX 재시도 실패: {n}/{len(payload)}")
                except Exception as e2:
                    self.get_logger().error(f"UART TX 재시도 예외: {e2}")

    # ---- ROS Callbacks
    def _on_done_msg(self, msg: String):
        """
        C++ 컨트롤러가 4번째 스텝에서 퍼블리시하는 "vacuum off"를 잡아
        MCU로 '3'을 전송(릴레이 토글 → OFF).
        """
        text = (msg.data or "").strip().lower()
        if text == 'vacuum off':
            self._send_byte('3', "OFF via done_topic")
        else:
            self.get_logger().debug(f"ignore done msg: '{msg.data}'")

    def _on_vacuum_cmd(self, msg: Bool):
        """
        True일 때만 '3' 송신(집기/시작). False는 전송하지 않음.
        OFF는 위의 "vacuum off" 시점에 처리.
        """
        if msg.data:
            self._send_byte('3', "ON via vacuum_cmd")
        else:
            self.get_logger().info("vacuum_cmd False (no UART TX)")

    # ---- Serial reading loop
    def _maybe_parse_csv(self, s: str) -> Optional[List[float]]:
        if not self.csv_hint or ',' not in s:
            return None
        try:
            vals = [float(x.strip()) for x in s.split(',') if x.strip() != '']
            return vals if len(vals) > 0 else None
        except Exception:
            return None

    def _try_publish_float(self, s: str):
        if not self.enable_float or self.pub_float is None:
            return
        try:
            value = float(s.strip())
            msg = Float32(); msg.data = value
            self.pub_float.publish(msg)
            self.get_logger().info(f"Float32 publish: {value:.4f}")
            return
        except ValueError:
            pass
        m = re.search(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', s)
        if m:
            try:
                value = float(m.group(0))
                msg = Float32(); msg.data = value
                self.pub_float.publish(msg)
                self.get_logger().info(f"Float32 publish(extracted): {value:.4f}")
            except ValueError:
                pass

    def _read_loop(self):
        while not self._stop.is_set():
            if self._ser is None or not self._ser.is_open:
                self._ser = self._open_serial()
                if self._ser is None:
                    break
            try:
                line = self._ser.readline()  # \n 기준
                if not line:
                    continue
                try:
                    data = line.decode(errors='ignore')
                except Exception:
                    continue
                data = data.replace('\r', '').replace('\n', '')
                if not data and not self.publish_empty:
                    continue
                msg = String(); msg.data = data
                self.pub_raw.publish(msg)
                self._try_publish_float(data)
                parsed = self._maybe_parse_csv(data)
                if parsed is not None:
                    self.get_logger().debug(f'CSV parsed: {parsed}')
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'시리얼 오류: {e}. 포트 재연결 시도...')
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
                time.sleep(self.reconnect_sec)
            except Exception as e:
                self.get_logger().warn(f'예외: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UartMultiReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
