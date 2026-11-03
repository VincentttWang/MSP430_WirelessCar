#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MSP430 & G3507 无线小车网页测控工作站本地服务 (Web Server & Serial Bridge)
项目: 智能小车双模测控平台
"""

import os
import sys
import json
import time
import socket
import struct
import threading
import webbrowser
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

PORT = 8080
HTML_DIR = os.path.dirname(os.path.abspath(__file__))

# 「串口已打开」与「小车真的在发帧」是两件事，必须分开上报：
#   g_is_connected : 端口是否成功打开（点"连接串口"就会 True）
#   g_telemetry["last_update"] : 最后一次收到**校验通过**的 20 字节帧的时刻
# 之前 /api/telemetry 直接返回 g_is_connected 当 connected，导致小车没开机
# 也显示"双向在线"——这是本次修复的 bug。
ONLINE_TIMEOUT_S = 1.0          # 超过 1s 没收到有效帧即判定离线

# 车辆串口与遥测状态
g_serial = None
g_serial_lock = threading.Lock()
g_is_connected = False
g_frame_count = 0
g_telemetry = {
    "speed": 0.0,
    "dist": 0.0,
    "yaw": 0.0,
    "acc": 1.0,
    "speed_pct": 50,
    "state": 0,
    "light": 0,
    "last_update": 0
}
g_running = True

def rx_thread_worker():
    """后台 20Hz 串口遥测帧读取线程"""
    global g_serial, g_is_connected, g_telemetry, g_running, g_frame_count
    buf = bytearray()
    last_hb_time = 0
    err_count = 0

    while g_running:
        if g_is_connected and g_serial and g_serial.is_open:
            try:
                # 200ms 心跳下发
                now = time.time()
                if (now - last_hb_time) >= 0.2:
                    last_hb_time = now
                    with g_serial_lock:
                        g_serial.write(b'h')

                # 读入可用数据
                n = g_serial.in_waiting
                err_count = 0
                if n > 0:
                    data = g_serial.read(n)
                    buf.extend(data)

                    while len(buf) >= 2:
                        if buf[0] == 0x21:  # '!'
                            end_idx = buf.find(b'#')
                            if end_idx != -1:
                                msg = bytes(buf[:end_idx+1]).decode('ascii', errors='ignore')
                                g_telemetry["calib_msg"] = msg
                                buf = buf[end_idx+1:]
                                continue
                            else:
                                if len(buf) > 128:
                                    buf.pop(0)
                                break
                        elif len(buf) >= 20 and buf[0] == 0xAA and buf[1] == 0x55:
                            if buf[18] == 0x55 and buf[19] == 0xAA:
                                chk = sum(buf[2:17]) & 0xFF
                                if chk == buf[17]:
                                    spd_pct = buf[2]
                                    # [3] 低 7 位是运行状态，最高位是"上位机在线"标志
                                    car_state = buf[3] & 0x7F
                                    link_online = bool(buf[3] & 0x80)
                                    light_state = buf[4]
                                    spd_mm = struct.unpack(">h", buf[5:7])[0]
                                    dist_mm = struct.unpack(">i", buf[7:11])[0]
                                    yaw_01 = struct.unpack(">h", buf[11:13])[0]
                                    acc_001g = struct.unpack(">h", buf[13:15])[0]

                                    g_telemetry = {
                                        "speed": spd_mm / 1000.0,
                                        "dist": dist_mm / 1000.0,
                                        "yaw": yaw_01 / 10.0,
                                        "acc": acc_001g / 100.0,
                                        "speed_pct": spd_pct,
                                        "state": car_state,
                                        "light": light_state,
                                        "car_link": link_online,
                                        "last_update": time.time(),
                                        "calib_msg": g_telemetry.get("calib_msg", "")
                                    }
                                    g_frame_count += 1
                                    buf = buf[20:]
                                    continue
                            buf = buf[1:]
                        else:
                            idx_aa = buf.find(0xAA)
                            idx_bang = buf.find(0x21)
                            nxt = -1
                            if idx_aa != -1 and idx_bang != -1:
                                nxt = min(idx_aa, idx_bang)
                            elif idx_aa != -1:
                                nxt = idx_aa
                            elif idx_bang != -1:
                                nxt = idx_bang

                            if nxt > 0:
                                buf = buf[nxt:]
                            elif nxt == -1:
                                buf.clear()
                            else:
                                break
            except Exception as e:
                print(f"[SERIAL ERR] 读取错误: {e}")
                # USB 拔掉后 read/in_waiting 会持续抛错，但 g_is_connected 仍是 True，
                # 前端就会永远显示"已连接"。连续失败 10 次即判定断开。
                err_count += 1
                if err_count >= 10:
                    print("[SERIAL ERR] 连续 10 次失败，判定串口已断开")
                    with g_serial_lock:
                        g_is_connected = False
                        try:
                            if g_serial:
                                g_serial.close()
                        except Exception:
                            pass
                        g_serial = None
                    err_count = 0
                time.sleep(0.05)
        time.sleep(0.005)

class GroundStationHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=HTML_DIR, **kwargs)

    def end_headers(self):
        # 允许跨域与无缓存
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        super().end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == '/api/ports':
            ports = []
            if HAS_SERIAL:
                for p in serial.tools.list_ports.comports():
                    ports.append({"device": p.device, "desc": p.description})
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(ports).encode('utf-8'))
            return

        elif path == '/api/info':
            """给"手机当手柄"用：返回局域网地址，PC 页面上直接显示让手机扫码/手输。"""
            ip = lan_ip()
            resp = {
                "port": PORT,
                "lan_ip": ip,
                "mobile_url": f"http://{ip}:{PORT}/mobile.html",
                "pc_url": f"http://localhost:{PORT}/index.html",
            }
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode('utf-8'))
            return

        elif path == '/api/telemetry':
            now = time.time()
            last = g_telemetry.get("last_update") or 0
            age_ms = round((now - last) * 1000.0, 1) if last > 0 else None
            # 只有"端口开着 + 最近真的收到过校验通过的帧"才算小车在线
            online = bool(g_is_connected and last > 0 and (now - last) <= ONLINE_TIMEOUT_S)

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            resp = {
                "connected": bool(g_is_connected),   # 串口是否已打开
                "online": online,                    # 小车是否真的在发帧
                "age_ms": age_ms,                    # 距最后一帧的毫秒数
                "frames": g_frame_count,             # 累计有效帧数
                "data": g_telemetry,
                "now": now
            }
            self.wfile.write(json.dumps(resp).encode('utf-8'))
            return

        # 默认静态页面分发
        return super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        length = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(length) if length > 0 else b''

        global g_serial, g_is_connected, g_frame_count

        if path == '/api/connect':
            try:
                payload = json.loads(post_data.decode('utf-8'))
                port = payload.get('port')
                baud = int(payload.get('baud', 115200))
                if HAS_SERIAL and port:
                    with g_serial_lock:
                        if g_serial and g_serial.is_open:
                            g_serial.close()
                        g_serial = serial.Serial(port, baud, timeout=0.05)
                        g_is_connected = True
                        # 关键：新连接的"最后一帧时刻"必须清零，
                        # 否则会沿用上一次连接的旧时间戳，一开串口就显示在线。
                        g_telemetry["last_update"] = 0
                        g_frame_count = 0
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"ok": True}).encode('utf-8'))
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"ok": False, "error": str(e)}).encode('utf-8'))
            return

        elif path == '/api/disconnect':
            with g_serial_lock:
                g_is_connected = False
                if g_serial:
                    try: g_serial.close()
                    except: pass
                    g_serial = None
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"ok": True}).encode('utf-8'))
            return

        elif path == '/api/send':
            try:
                payload = json.loads(post_data.decode('utf-8'))
                cmd = payload.get('cmd', '')
                if g_is_connected and g_serial and g_serial.is_open and cmd:
                    with g_serial_lock:
                        g_serial.write(cmd.encode('ascii'))
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"ok": True}).encode('utf-8'))
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"ok": False, "error": str(e)}).encode('utf-8'))
            return

        self.send_response(404)
        self.end_headers()

def lan_ip():
    """取本机局域网 IPv4。UDP connect 不实际发包，只是让内核查一次路由表，
    这样拿到的是"手机能访问到的那个地址"，而不是 127.0.0.1。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    except Exception:
        return '127.0.0.1'
    finally:
        s.close()


def main():
    threading.Thread(target=rx_thread_worker, daemon=True).start()

    server_address = ('', PORT)
    # 必须多线程：手机和 PC 会同时以 20Hz 轮询 /api/telemetry，
    # 单线程 HTTPServer 会被一个慢连接堵住所有人。
    httpd = ThreadingHTTPServer(server_address, GroundStationHandler)
    url = f"http://localhost:{PORT}/index.html"
    ip = lan_ip()
    mobile = f"http://{ip}:{PORT}/mobile.html"
    print("=" * 64)
    print("🚀 MSP430 网页测控工作站已启动!")
    print(f"👉 PC 端:   {url}")
    print(f"👉 手机手柄: {mobile}   (手机与电脑连同一个 Wi-Fi)")
    print("提示: 支持 Chrome / Edge Web Serial 原生直连与 Python 服务桥接双模")
    print("=" * 64)

    # 自动弹出浏览器
    threading.Timer(0.8, lambda: webbrowser.open(url)).start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n服务已停止。")
        global g_running
        g_running = False

if __name__ == "__main__":
    main()
