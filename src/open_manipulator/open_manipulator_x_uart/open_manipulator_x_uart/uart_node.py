#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
from typing import Optional, List
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None  # pyserial 미설치 대비

class UartMultiReceiver(Node):
    """
    - UART에서 한 줄(line)씩 읽어서:
      1) raw 문자열을 String 토픽으로 퍼블리시
      2) 동일 라인을 float으로 변환 가능하면 Float32 토픽으로도 퍼블리시
    - CSV처럼 보이면 디버그 로그에 파싱 결과 출력(옵션)
    - 포트 오류 시 자동 재연결
    - (NEW) 완료/상태 토픽 구독 → UART로 지정 페이로드 전송
      * "vacuum off" 수신 시 '3' 전송
      * 그 외 메시지는 기존 기본 페이로드(기본 '2') 전송
    """

    def __init__(self):
        super().__init__('uart_multi_receiver')

        # --- ROS 파라미터 (런치/명령행에서 변경 가능)
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('newline', '\n')              # 개행 기준(참고용)
        self.declare_parameter('reconnect_sec', 3.0)         # 끊기면 재연결 주기
        self.declare_parameter('publish_empty_lines', False)

        # 퍼블리시 관련
        self.declare_parameter('topic_raw', 'nucleo_uart')   # 문자열 토픽
        self.declare_parameter('csv_hint', True)             # CSV처럼 보이면 숫자 파싱해 로그
        self.declare_parameter('enable_float', True)         # 숫자 1개면 Float32 퍼블리시
        self.declare_parameter('topic_float', '/ultrasonic_distance')

        # === 완료/상태 토픽 수신 → UART 송신
        self.declare_parameter('done_topic', '/manipulator_done')
        self.declare_parameter('tx_on_done', True)           # 완료 수신 시 전송 여부
        self.declare_parameter('tx_payload_on_done', '2')    # 기본 전송 페이로드
        self.declare_parameter('tx_append_newline', True)    # \n 붙여 보낼지

        # 파라미터 값 읽기
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.newline = self.get_parameter('newline').get_parameter_value().string_value
        self.reconnect_sec = float(self.get_parameter('reconnect_sec').get_parameter_value().double_value)
        self.publish_empty = self.get_parameter('publish_empty_lines').get_parameter_value().bool_value

        self.topic_raw = self.get_parameter('topic_raw').get_parameter_value().string_value
        self.csv_hint = self.get_parameter('csv_hint').get_parameter_value().bool_value
        self.enable_float = self.get_parameter('enable_float').get_parameter_value().bool_value
        self.topic_float = self.get_parameter('topic_float').get_parameter_value().string_value
        
        # 완료/상태 토픽 관련 파라미터
        self.done_topic = self.get_parameter('done_topic').get_parameter_value().string_value
        self.tx_on_done = self.get_parameter('tx_on_done').get_parameter_value().bool_value
        self.tx_payload_on_done = self.get_parameter('tx_payload_on_done').get_parameter_value().string_value
        self.tx_append_newline = self.get_parameter('tx_append_newline').get_parameter_value().bool_value

        if serial is None:
            self.get_logger().fatal('pyserial이 설치되지 않았습니다. `pip install pyserial` 후 다시 실행해주세요.')
            raise SystemExit(1)

        # 퍼블리셔
        self.pub_raw = self.create_publisher(String, self.topic_raw, 10)
        self.pub_float = (self.create_publisher(Float32, self.topic_float, 10) if self.enable_float else None)

        # 완료/상태 토픽 구독 → UART 전송 콜백
        self.sub_done = self.create_subscription(
            String, self.done_topic, self._on_done_msg, 10
        )

        # 시리얼 스레드
        self._ser: Optional['serial.Serial'] = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()   # TX 보호용 락
        self._th = threading.Thread(target=self._read_loop, daemon=True)
        self._th.start()

        self.get_logger().info(
            f'UART Multi Receiver started: port={self.port}, baud={self.baud}, '
            f'raw_topic={self.topic_raw}, float_topic={self.topic_float if self.enable_float else "disabled"}, '
            f'done_topic={self.done_topic}, tx_on_done={self.tx_on_done}'
        )

    # 안전 종료
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

    # 포트 열기 (없으면 재시도)
    def _open_serial(self) -> Optional['serial.Serial']:
        while not self._stop.is_set():
            try:
                ser = serial.Serial(
                    self.port,
                    self.baud,
                    timeout=1.0,        # 읽기 타임아웃
                    write_timeout=1.0
                )
                self.get_logger().info(f'Connected to {self.port} @ {self.baud}')
                return ser
            except Exception as e:
                self.get_logger().warn(f'포트 열기 실패: {e}. {self.reconnect_sec:.1f}s 후 재시도...')
                time.sleep(self.reconnect_sec)
        return None

    # CSV 힌트가 있으면 숫자 파싱 시도 (디버그용)
    def _maybe_parse_csv(self, s: str) -> Optional[List[float]]:
        if not self.csv_hint:
            return None
        if ',' not in s:
            return None
        try:
            vals = [float(x.strip()) for x in s.split(',') if x.strip() != '']
            return vals if len(vals) > 0 else None
        except Exception:
            return None

    def _try_publish_float(self, s: str):
        if not self.enable_float or self.pub_float is None:
            return
        # 1) 먼저 전체가 숫자인지 시도
        try:
            value = float(s.strip())
            msg = Float32()
            msg.data = value
            self.pub_float.publish(msg)
            self.get_logger().info(f"Float32 publish: {value:.4f}")
            return
        except ValueError:
            pass

        # 2) 문자열 내부에서 첫 번째 부동소수 추출 (예: "Distance: 12.34 cm")
        m = re.search(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', s)
        if m:
            try:
                value = float(m.group(0))
                msg = Float32()
                msg.data = value
                self.pub_float.publish(msg)
                self.get_logger().info(f"Float32 publish(extracted): {value:.4f}")
            except ValueError:
                pass

    # 완료/상태 토픽 콜백: UART로 페이로드 전송
    def _on_done_msg(self, msg: String):
        text = (msg.data or "").strip().lower()
        # 오직 "vacuum off"만 처리, 나머지는 무시해서 '2'가 안 나가게 함
        if text != 'vacuum off':
            self.get_logger().debug(f"ignore done msg: '{msg.data}'")
            return

        payload = '3'
        if self.tx_append_newline:
            payload += '\n'

        try:
            with self._write_lock:
                if self._ser is None or not self._ser.is_open:
                    self._ser = self._open_serial()
                    if self._ser is None:
                        self.get_logger().error("UART 미연결 상태라 '3' 전송 실패")
                        return
                self._ser.write(payload.encode('utf-8'))
                self.get_logger().info("UART TX (vacuum off): '3'")
        except Exception as e:
            self.get_logger().error(f"UART TX 실패: {e}")

    # 읽기 스레드
    def _read_loop(self):
        while not self._stop.is_set():
            if self._ser is None or not self._ser.is_open:
                self._ser = self._open_serial()
                if self._ser is None:
                    break  # 종료

            try:
                line = self._ser.readline()  # \n 기준
                if not line:
                    continue

                try:
                    data = line.decode(errors='ignore')
                except Exception:
                    continue

                # 개행 제거
                data = data.replace('\r', '').replace('\n', '')
                if not data and not self.publish_empty:
                    continue

                # 1) raw 문자열 퍼블리시
                msg = String()
                msg.data = data
                self.pub_raw.publish(msg)

                # 2) 숫자 1개면 Float32 퍼블리시
                self._try_publish_float(data)

                # 3) CSV 파싱 로그(선택)
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
