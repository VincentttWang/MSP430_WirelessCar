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

PORT = int(os.environ.get('MSP430_PORT', '8080'))
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
                                g_telemetry["calib_seq"] = g_telemetry.get("calib_seq", 0) + 1
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
                                        "calib_msg": g_telemetry.get("calib_msg", ""),
                                        "calib_seq": g_telemetry.get("calib_seq", 0)
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
            """给"手机当手柄"用：把所有可达地址都返回，校园网 IPv4 不通时可选 IPv6。"""
            entries = address_entries()
            ip4 = lan_ip()
            mobile_url = url_for(ip4)
            preferred = entries[0]["url"] if entries else mobile_url
            is_hotspot = any(a["kind"] == "ipv4-hotspot" for a in entries)
            if is_hotspot:
                mobile_url = preferred          # 旧页面只读 mobile_url，也要给对
            resp = {
                "port": PORT,
                "lan_ip": ip4,
                "hotspot": is_hotspot,
                "mobile_url": mobile_url,
                "preferred_url": preferred,
                "addresses": entries,
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


def url_for(ip, page='mobile.html'):
    """IPv6 地址在 URL 里必须加方括号，否则冒号会被当成端口分隔符。"""
    host = f'[{ip}]' if ':' in ip else ip
    return f'http://{host}:{PORT}/{page}'


def _temporary_v6():
    """Windows 隐私扩展生成的「临时 IPv6」会定期变，不适合当推荐地址，降权到末尾。"""
    if os.name != 'nt':
        return set()
    try:
        import re
        import subprocess
        raw = subprocess.check_output(['ipconfig', '/all'], timeout=2,
                                      stderr=subprocess.DEVNULL)
        text = raw.decode('mbcs', errors='ignore')
        found = set()
        for line in text.splitlines():
            if 'Temporary' in line or '临时' in line:
                cand = re.findall(r'(?:[0-9a-fA-F]{1,4}:){2,}[0-9a-fA-F]{1,4}', line)
                if cand:
                    found.add(cand[-1].lower())
        return found
    except Exception:
        return set()


def local_addresses():
    """枚举本机全部可用地址，分成 (全球 IPv6, 局域网 IPv4)。

    校园网常见情况：IPv4 是 10.x 私网、AP 隔离 / 跨网段不通，
    但 IPv6 是全球单播且端到端可达 —— 所以两个都要给出来让用户试。
    link-local(fe80::) 与 loopback 不能跨设备访问，直接过滤。
    """
    ips = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None,
                                       socket.AF_UNSPEC, socket.SOCK_STREAM):
            ip = (info[4][0] or '').split('%')[0]
            if ip:
                ips.add(ip)
    except OSError:
        pass
    try:
        ips.add(lan_ip())
    except OSError:
        pass

    v6, v4 = [], []
    for ip in sorted(ips):
        low = ip.lower()
        if ':' in ip:
            if low.startswith(('fe80:', '::1', 'fc', 'fd')):
                continue
            v6.append(ip)
        else:
            if low.startswith(('127.', '169.254.')):
                continue
            v4.append(ip)
    temp = _temporary_v6()
    v6.sort(key=lambda ip: (ip.lower() in temp, ip))
    return v6, v4


# 常见热点网段：Windows 移动热点 / Android Wi-Fi 热点 / iPhone 个人热点 / USB 共享
HOTSPOT_V4_PREFIXES = ('192.168.137.', '192.168.43.', '192.168.42.',
                       '192.168.49.', '172.20.10.')


def is_hotspot_v4(ip):
    """判断某个 IPv4 是否来自热点网卡。热点里 IPv4 直连最可靠，IPv6 全球地址在该链路无效。"""
    return ip.startswith(HOTSPOT_V4_PREFIXES)


def address_entries():
    """统一给 /api/info 与启动横幅用的地址清单（按当前网络形态自动排序）。"""
    v6, v4 = local_addresses()
    temp = _temporary_v6()
    hotspot = [ip for ip in v4 if is_hotspot_v4(ip)]
    v4_rest = [ip for ip in v4 if ip not in hotspot]
    out = []
    if hotspot:
        for ip in hotspot:
            out.append({"ip": ip, "url": url_for(ip), "kind": "ipv4-hotspot",
                        "label": "热点直连（手机/PC 开热点时用这条，推荐）"})
        for ip in v4_rest:
            out.append({"ip": ip, "url": url_for(ip), "kind": "ipv4",
                        "label": "IPv4 局域网（另一张网卡，可能不通）"})
        for ip in v6:
            out.append({"ip": ip, "url": url_for(ip), "kind": "ipv6",
                        "label": "IPv6 校园网直连（热点下通常不可用）"})
        return out

    for idx, ip in enumerate(v6):
        if ip.lower() in temp:
            label = "IPv6 临时地址（会变，备选）"
        elif idx == 0:
            label = "IPv6 直连（校园网首选，稳定）"
        else:
            label = "IPv6 直连（备选）"
        out.append({"ip": ip, "url": url_for(ip), "kind": "ipv6", "label": label})
    for ip in v4_rest:
        out.append({"ip": ip, "url": url_for(ip), "kind": "ipv4", "label": "IPv4 局域网"})
    return out


def make_server(family):
    """按协议族起一个多线程 HTTP 服务；IPv6 强制 V6ONLY，避免和 IPv4 抢端口。"""
    class _ThreadedServer(ThreadingHTTPServer):
        address_family = family
        allow_reuse_address = True
        daemon_threads = True

    if family == socket.AF_INET6:
        class _V6Server(_ThreadedServer):
            def server_bind(self):
                try:
                    self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
                except (AttributeError, OSError):
                    pass
                super().server_bind()
        return _V6Server(('::', PORT), GroundStationHandler)
    return _ThreadedServer(('', PORT), GroundStationHandler)


def main():
    global g_running
    threading.Thread(target=rx_thread_worker, daemon=True).start()

    servers = []
    for family, label in ((socket.AF_INET, 'IPv4'), (socket.AF_INET6, 'IPv6')):
        try:
            srv = make_server(family)
        except OSError as e:
            print(f"⚠ {label} 监听失败: {e}（另一协议仍可用）")
            continue
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        servers.append((label, srv))
    if not servers:
        print(f"❌ 端口 {PORT} 两个协议都绑不上，检查是否被占用/权限")
        return

    addr_list = address_entries()
    entries = [(a["label"], a["url"]) for a in addr_list]
    print("=" * 72)
    print("🚀 MSP430 网页测控工作站已启动!")
    print(f"👉 PC 端:   http://localhost:{PORT}/index.html")
    print(f"已监听: {', '.join(l for l, _ in servers)}  ·  端口 {PORT}")
    print("手机手柄地址（按可达性从高到低试，第一个能打开就用它）:")
    if addr_list and addr_list[0]["kind"] == "ipv4-hotspot":
        print("   检测到热点网段：手机开热点 / PC 连手机热点时，直接用第 1 条 IPv4 即可，无需穿透")
    if entries:
        for i, (label, url) in enumerate(entries):
            print(f"   {i + 1}. [{label}] {url}")
            print(f"      全屏自动驾驶: {url}#auto")
    else:
        print("   ⚠ 没找到局域网地址，可用 PC 开热点 / USB 共享网络后重启本服务")
    print(f"若都打不开：① 放行防火墙 TCP 入站 {PORT}  ② 校园网 AP 隔离时改开热点或 USB(adb reverse)")
    print("=" * 72)

    # 自动弹出浏览器
    threading.Timer(0.8, lambda: webbrowser.open(f'http://localhost:{PORT}/index.html')).start()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n服务已停止。")
        g_running = False
        for _, srv in servers:
            srv.shutdown()

if __name__ == "__main__":
    main()
