#!/usr/bin/env python3
"""
底盘控制器模拟器 — 模拟 STM32 底盘
通过 TCP socket 与 motor_driver 通信。

协议：
  控制帧 (PC→底盘): 16字节 0xAA + 0x01 + vx(f32 LE) + wz(f32 LE) + 使能 + 保留4 + CS
  反馈帧 (底盘→PC): 25字节 0xBB + 0x02 + 左里程(f32) + 右里程(f32) + 电压(f32) + 故障码 + 保留9 + CS

用法:
  python simulation/chassis_simulator.py --port 9999
  python simulation/chassis_simulator.py --port 9999 --vx 0.3  --fault-cycle 30
"""
import argparse
import struct
import socket
import threading
import time
import math
import sys

# ── Protocol Constants ──
CONTROL_HEADER  = 0xAA
CONTROL_CMD     = 0x01
CONTROL_SIZE    = 16

FEEDBACK_HEADER = 0xBB
FEEDBACK_CMD    = 0x02
FEEDBACK_SIZE   = 25

# ── Checksum ──
def checksum(data: bytes) -> int:
    return sum(data) & 0xFF

# ── Frame Decode (Control) ──
def decode_control(frame: bytes):
    """Return (vx, wz, motor_enable) or None."""
    if len(frame) < CONTROL_SIZE:
        return None
    if frame[0] != CONTROL_HEADER or frame[1] != CONTROL_CMD:
        return None
    if frame[CONTROL_SIZE - 1] != checksum(frame[:CONTROL_SIZE - 1]):
        return None
    vx = struct.unpack_from('<f', frame, 2)[0]
    wz = struct.unpack_from('<f', frame, 6)[0]
    enable = bool(frame[10])
    return vx, wz, enable

# ── Frame Encode (Feedback) ──
def encode_feedback(left_m: float, right_m: float, voltage: float, fault: int) -> bytes:
    buf = bytearray(FEEDBACK_SIZE)
    buf[0] = FEEDBACK_HEADER
    buf[1] = FEEDBACK_CMD
    struct.pack_into('<f', buf, 2, left_m)
    struct.pack_into('<f', buf, 6, right_m)
    struct.pack_into('<f', buf, 10, voltage)
    buf[14] = fault & 0xFF
    # bytes 15-23 reserved, already 0x00
    buf[24] = checksum(buf[:24])
    return bytes(buf)

# ── Chassis Simulator ──
class ChassisSimulator:
    def __init__(self, host='127.0.0.1', port=9999,
                 tick_hz=50, fault_cycle=0):
        self.host = host
        self.port = port
        self.tick_hz = tick_hz
        self.tick_s = 1.0 / tick_hz
        self.fault_cycle = fault_cycle  # 0 = never fault

        # Simulated state
        self.left_m  = 0.0
        self.right_m = 0.0
        self.voltage = 12.5
        self.fault   = 0
        self.motor_enable = False
        self.vx = 0.0
        self.wz = 0.0
        self.tick_count = 0

        self.server = None
        self.clients = []
        self.running = False

    def start(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen(1)
        self.running = True
        print(f"[底盘模拟器] 监听 {self.host}:{self.port}")

        # Accept thread
        accept_t = threading.Thread(target=self._accept_loop, daemon=True)
        accept_t.start()

        # Main simulation loop
        self._sim_loop()

    def _accept_loop(self):
        while self.running:
            try:
                conn, addr = self.server.accept()
                print(f"[底盘模拟器] 客户端连接: {addr}")
                conn.settimeout(1.0)
                self.clients.append(conn)
                # Reader thread for this client
                rt = threading.Thread(target=self._client_reader, args=(conn, addr), daemon=True)
                rt.start()
            except socket.timeout:
                continue
            except OSError:
                break

    def _client_reader(self, conn, addr):
        buf = b''
        while self.running:
            try:
                data = conn.recv(256)
                if not data:
                    print(f"[底盘模拟器] 客户端断开: {addr}")
                    break
                buf += data
                # Parse complete control frames
                while len(buf) >= CONTROL_SIZE:
                    # Search for header
                    idx = buf.find(bytes([CONTROL_HEADER]))
                    if idx < 0:
                        buf = b''
                        break
                    if idx > 0:
                        buf = buf[idx:]
                        continue
                    if len(buf) < CONTROL_SIZE:
                        break
                    frame = buf[:CONTROL_SIZE]
                    result = decode_control(frame)
                    if result is not None:
                        self.vx, self.wz, self.motor_enable = result
                        self.tick_count += 1
                        if self.tick_count % max(1, 50 // self.tick_hz) == 0:
                            self._print_status()
                    else:
                        # Bad frame - skip header byte
                        buf = buf[1:]
                        continue
                    buf = buf[CONTROL_SIZE:]
            except socket.timeout:
                continue
            except OSError:
                break
        self._remove_client(conn)

    def _remove_client(self, conn):
        try:
            conn.close()
        except OSError:
            pass
        if conn in self.clients:
            self.clients.remove(conn)

    def _print_status(self):
        en = "ON " if self.motor_enable else "OFF"
        print(f"  vx={self.vx:.3f}  wz={self.wz:.3f}  en={en}  "
              f"里程=({self.left_m:.2f},{self.right_m:.2f})  "
              f"电压={self.voltage:.2f}V  fault=0x{self.fault:02X}")

    def _sim_loop(self):
        while self.running:
            start = time.time()

            # Update simulation state
            if self.motor_enable:
                # Wheel speeds from differential drive kinematics
                wheel_base = 0.178
                left_speed  = self.vx - self.wz * wheel_base / 2.0
                right_speed = self.vx + self.wz * wheel_base / 2.0
                self.left_m  += left_speed  * self.tick_s
                self.right_m += right_speed * self.tick_s

            # Battery: slow discharge
            self.voltage = max(10.0, self.voltage - 0.0001 * self.tick_s / 0.02)

            # Fault cycle
            if self.fault_cycle > 0 and self.tick_count % self.fault_cycle == 0 and self.tick_count > 0:
                self.fault = 0x01  # simulate overcurrent
            else:
                self.fault = 0x00

            # Send feedback to all connected clients
            fb = encode_feedback(self.left_m, self.right_m,
                                 self.voltage, self.fault)
            dead_clients = []
            for c in self.clients:
                try:
                    c.sendall(fb)
                except OSError:
                    dead_clients.append(c)
            for c in dead_clients:
                self._remove_client(c)

            # Maintain tick rate
            elapsed = time.time() - start
            sleep = self.tick_s - elapsed
            if sleep > 0:
                time.sleep(sleep)

    def stop(self):
        self.running = False
        for c in self.clients[:]:
            self._remove_client(c)
        if self.server:
            self.server.close()
        print("[底盘模拟器] 已停止")


def main():
    parser = argparse.ArgumentParser(description='底盘控制器模拟器')
    parser.add_argument('--host', default='127.0.0.1', help='监听地址')
    parser.add_argument('--port', type=int, default=9999, help='监听端口')
    parser.add_argument('--hz', type=int, default=50, help='反馈帧频率 (Hz)')
    parser.add_argument('--fault-cycle', type=int, default=0,
                        help='每隔 N 帧触发一次故障码 (0=从不)')
    args = parser.parse_args()

    sim = ChassisSimulator(
        host=args.host, port=args.port,
        tick_hz=args.hz, fault_cycle=args.fault_cycle,
    )
    try:
        sim.start()
    except KeyboardInterrupt:
        print("\n[底盘模拟器] 收到中断")
    finally:
        sim.stop()


if __name__ == '__main__':
    main()
