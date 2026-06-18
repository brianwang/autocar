#!/usr/bin/env python3
"""
motor_driver 协议测试客户端 + 底盘模拟器一体启动
用法:
  python3 motor_test.py                    # 自动启动模拟器 + 连接
  python3 motor_test.py --sim-only         # 只跑模拟器
  python3 motor_test.py --client-only      # 只跑客户端（连接已有模拟器）
"""
import argparse
import socket
import struct
import threading
import time
import os
import sys
import signal

# ── Protocol Constants ──
CONTROL_HEADER   = 0xAA
CONTROL_CMD      = 0x01
CONTROL_SIZE     = 16
FEEDBACK_HEADER  = 0xBB
FEEDBACK_CMD     = 0x02
FEEDBACK_SIZE    = 25

def checksum(data: bytes) -> int:
    return sum(data) & 0xFF

def encode_control(vx: float, wz: float, enable: bool = True) -> bytes:
    buf = bytearray(CONTROL_SIZE)
    buf[0] = CONTROL_HEADER
    buf[1] = CONTROL_CMD
    struct.pack_into('<f', buf, 2, vx)
    struct.pack_into('<f', buf, 6, wz)
    buf[10] = 0x01 if enable else 0x00
    buf[15] = checksum(buf[:15])
    return bytes(buf)

def decode_feedback(frame: bytes):
    if len(frame) < FEEDBACK_SIZE:
        return None
    if frame[0] != FEEDBACK_HEADER or frame[1] != FEEDBACK_CMD:
        return None
    if frame[24] != checksum(frame[:24]):
        return None
    lm = struct.unpack_from('<f', frame, 2)[0]
    rm = struct.unpack_from('<f', frame, 6)[0]
    bat = struct.unpack_from('<f', frame, 10)[0]
    fault = frame[14]
    return lm, rm, bat, fault


# ── log colors ──
GREEN  = '\033[92m'
BLUE   = '\033[94m'
YELLOW = '\033[93m'
RED    = '\033[91m'
CYAN   = '\033[96m'
RESET  = '\033[0m'
BOLD   = '\033[1m'

def tx_log(vx, wz, enable):
    t = time.time()
    ms = int((t - int(t)) * 1000)
    ts = time.strftime('%H:%M:%S') + f'.{ms:03d}'
    en = f'{GREEN}EN{RESET}' if enable else f'{RED}DIS{RESET}'
    print(f'{CYAN}{ts} TX ➤{RESET}  vx={vx:+.3f}  wz={wz:+.3f}  {en}')

def rx_log(lm, rm, bat, fault):
    t = time.time()
    ms = int((t - int(t)) * 1000)
    ts = time.strftime('%H:%M:%S') + f'.{ms:03d}'
    fault_str = f'  {RED}FAULT=0x{fault:02X}{RESET}' if fault else ''
    print(f'{YELLOW}{ts} RX ◀{RESET}  L={lm:+.4f}  R={rm:+.4f}  '
          f'bat={bat:.2f}V{fault_str}')


# ── Simulator (embedded) ──
class ChassisSimulator:
    def __init__(self, host='127.0.0.1', port=9999, tick_hz=50):
        self.host = host
        self.port = port
        self.tick_hz = tick_hz
        self.tick_s = 1.0 / tick_hz
        self.left_m = 0.0
        self.right_m = 0.0
        self.voltage = 12.5
        self.fault = 0
        self.vx = 0.0
        self.wz = 0.0
        self.motor_enable = False
        self.tick_count = 0
        self.running = False
        self.server = None
        self.clients = []
        self.wheel_base = 0.178

    def start(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen(1)
        self.running = True
        print(f'  {BOLD}[SIM]{RESET} 监听 {self.host}:{self.port}')

        threading.Thread(target=self._accept_loop, daemon=True).start()
        threading.Thread(target=self._sim_loop, daemon=True).start()

    def _accept_loop(self):
        while self.running:
            try:
                conn, addr = self.server.accept()
                print(f'  {BOLD}[SIM]{RESET} 客户端连接: {addr}')
                conn.settimeout(1.0)
                self.clients.append(conn)
                threading.Thread(target=self._client_reader, args=(conn, addr), daemon=True).start()
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
                    break
                buf += data
                while len(buf) >= CONTROL_SIZE:
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
                    if frame[0] == CONTROL_HEADER and frame[1] == CONTROL_CMD:
                        if frame[15] == checksum(frame[:15]):
                            vx = struct.unpack_from('<f', frame, 2)[0]
                            wz = struct.unpack_from('<f', frame, 6)[0]
                            self.vx = vx
                            self.wz = wz
                            self.motor_enable = bool(frame[10])
                            self.tick_count += 1
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

    def _sim_loop(self):
        while self.running:
            start_t = time.time()
            if self.motor_enable:
                ls = self.vx - self.wz * self.wheel_base / 2.0
                rs = self.vx + self.wz * self.wheel_base / 2.0
                self.left_m  += ls * self.tick_s
                self.right_m += rs * self.tick_s
            self.voltage = max(10.0, self.voltage - 0.0001 * self.tick_s / 0.02)
            if self.tick_count > 0 and self.tick_count % 200 == 0:
                self.fault = 0x01
            else:
                self.fault = 0x00
            fb = bytearray(FEEDBACK_SIZE)
            fb[0] = FEEDBACK_HEADER
            fb[1] = FEEDBACK_CMD
            struct.pack_into('<f', fb, 2, self.left_m)
            struct.pack_into('<f', fb, 6, self.right_m)
            struct.pack_into('<f', fb, 10, self.voltage)
            fb[14] = self.fault
            fb[24] = checksum(fb[:24])
            dead = []
            for c in self.clients:
                try:
                    c.sendall(bytes(fb))
                except OSError:
                    dead.append(c)
            for c in dead:
                self._remove_client(c)
            el = time.time() - start_t
            if self.tick_s - el > 0:
                time.sleep(self.tick_s - el)

    def stop(self):
        self.running = False
        for c in self.clients[:]:
            self._remove_client(c)
        if self.server:
            self.server.close()


# ── Motor Protocol Client ──
def run_client(host, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3)
    sock.connect((host, port))
    print(f'  {BOLD}[CLI]{RESET} 已连接 {host}:{port}')
    sock.settimeout(0.5)

    # test sequence generator
    def test_seq():
        phases = [
            (0.2, 0.0, 3.0,   '前进 0.2m/s'),
            (0.0, 0.0, 1.0,   '停止'),
            (0.0, 0.5, 2.0,   '左转 wz=0.5'),
            (0.0, 0.0, 1.0,   '停止'),
            (0.3, 0.0, 2.0,   '加速 0.3m/s'),
            (0.0, -0.3, 2.0,  '右转 wz=-0.3'),
            (-0.15, 0.0, 2.0, '后退 0.15m/s'),
            (0.0, 0.0, 1.0,   '停止'),
        ]
        for vx, wz, dur, desc in phases:
            yield vx, wz, dur, desc

    recv_buf = b''
    running = [True]  # use list for mutability in callback
    def on_sigint(*_):
        running[0] = False
    signal.signal(signal.SIGINT, on_sigint)

    for vx, wz, dur, desc in test_seq():
        if not running[0]:
            break
        print(f'\n  --- {BOLD}{desc}{RESET} ---')
        t_start = time.time()
        while time.time() - t_start < dur:
            if not running[0]:
                break
            # TX
            frame = encode_control(vx, wz)
            tx_log(vx, wz, True)
            try:
                sock.sendall(frame)
            except OSError as e:
                print(f'  {RED}TX ERROR:{RESET} {e}')
                running[0] = False
                break

            # RX
            try:
                chunk = sock.recv(1024)
                if chunk:
                    recv_buf += chunk
                    while len(recv_buf) >= FEEDBACK_SIZE:
                        found = False
                        for i in range(len(recv_buf) - FEEDBACK_SIZE + 1):
                            res = decode_feedback(recv_buf[i:i+FEEDBACK_SIZE])
                            if res:
                                rx_log(*res)
                                recv_buf = recv_buf[i+FEEDBACK_SIZE:]
                                found = True
                                break
                        if not found:
                            # skip bad byte
                            recv_buf = recv_buf[1:]
            except socket.timeout:
                pass
            except OSError as e:
                print(f'  {RED}RX ERROR:{RESET} {e}')
                break

            time.sleep(0.02)  # 50Hz

    sock.close()
    print(f'\n  {BOLD}[CLI]{RESET} 断开')


def main():
    parser = argparse.ArgumentParser(description='底盘协议测试')
    parser.add_argument('--host', default='127.0.0.1', help='模拟器地址')
    parser.add_argument('--port', type=int, default=9999, help='模拟器端口')
    parser.add_argument('--sim-only', action='store_true', help='只启动模拟器')
    parser.add_argument('--client-only', action='store_true', help='只启动客户端')
    args = parser.parse_args()

    sim = None
    if not args.client_only:
        sim = ChassisSimulator(host=args.host, port=args.port)
        sim.start()
        print(f'{GREEN}═══ 底盘模拟器已启动 ═══{RESET}')
        if args.sim_only:
            print('按 Ctrl+C 停止')
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                pass
            sim.stop()
            return

    if not args.sim_only:
        if sim:
            time.sleep(0.3)  # wait for sim to be ready
        print(f'{GREEN}═══ 协议测试客户端启动 ═══{RESET}')
        print(f'{GREEN}    模拟器: {args.host}:{args.port}{RESET}')
        print(f'{GREEN}    退出: Ctrl+C{RESET}')
        print()
        try:
            run_client(args.host, args.port)
        except KeyboardInterrupt:
            pass

    if sim:
        sim.stop()
    print(f'{GREEN}═══ 已停止 ═══{RESET}')


if __name__ == '__main__':
    main()
