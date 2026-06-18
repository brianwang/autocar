#!/usr/bin/env python3
"""Connect to chassis simulator, send/receive frames, print traffic."""
import socket
import struct
import time

def checksum(data):
    return sum(data) & 0xFF

HOST = "127.0.0.1"
PORT = 9999

print("=== Chassis Simulator Traffic Test ===\n")

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(3)
sock.connect((HOST, PORT))
print(f"Connected to {HOST}:{PORT}\n")

# Phase 1: Forward
print("--- Phase 1: Forward 0.2 m/s ---")
for i in range(5):
    buf = bytearray(16)
    buf[0] = 0xAA
    buf[1] = 0x01
    struct.pack_into("<f", buf, 2, 0.2)
    struct.pack_into("<f", buf, 6, 0.0)
    buf[10] = 0x01
    buf[15] = checksum(buf[:15])

    sock.sendall(bytes(buf))
    print(f"TX [{i+1}] vx=+0.200  wz=+0.000  EN")

    time.sleep(0.05)
    try:
        fb = sock.recv(256)
        if len(fb) >= 25 and fb[0] == 0xBB:
            lm = struct.unpack_from("<f", fb, 2)[0]
            rm = struct.unpack_from("<f", fb, 6)[0]
            bat = struct.unpack_from("<f", fb, 10)[0]
            fault = fb[14]
            cs_ok = "OK" if fb[24] == checksum(fb[:24]) else "BAD"
            print(f"RX [{i+1}] L={lm:+.4f}  R={rm:+.4f}  bat={bat:.2f}V  fault=0x{fault:02X}  CS={cs_ok}")
    except socket.timeout:
        print(f"RX [{i+1}] (timeout)")

# Phase 2: Turn
print("\n--- Phase 2: Turn wz=0.5 ---")
for i in range(5):
    buf = bytearray(16)
    buf[0] = 0xAA
    buf[1] = 0x01
    struct.pack_into("<f", buf, 2, 0.0)
    struct.pack_into("<f", buf, 6, 0.5)
    buf[10] = 0x01
    buf[15] = checksum(buf[:15])

    sock.sendall(bytes(buf))
    print(f"TX [{i+1}] vx=+0.000  wz=+0.500  EN")

    time.sleep(0.05)
    try:
        fb = sock.recv(256)
        if len(fb) >= 25 and fb[0] == 0xBB:
            lm = struct.unpack_from("<f", fb, 2)[0]
            rm = struct.unpack_from("<f", fb, 6)[0]
            bat = struct.unpack_from("<f", fb, 10)[0]
            diff = rm - lm
            print(f"RX [{i+1}] L={lm:+.4f}  R={rm:+.4f}  diff={diff:+.4f}  bat={bat:.2f}V")
    except socket.timeout:
        print(f"RX [{i+1}] (timeout)")

sock.close()
print("\n=== Test Complete ===")
