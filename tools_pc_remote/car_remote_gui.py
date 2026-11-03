#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MSP430 & G3507 无线小车双模测控工作站 (PC 上位机)
智能小车测控平台
功能：
1. 自动枚举与连接无线 DAP / USB-TTL 串口 / 蓝牙 / MSP-EXP430F5529
2. 键盘 (W/A/S/D/空格/1/2/3/+/ -/R/G/Z) 与图形化按键双模操控
3. 双向速度滑块调速 (0% ~ 100%) 与即时生效联动
4. 20 字节高频二进制遥测解包：线速度、总里程、航向角 (Yaw)、总加速度 (Acc)、交通灯状态
5. 精准双向连接检测与心跳维持 (200ms 心跳包保持小车绿灯常亮与在线指示)
"""

import sys
import time
import struct
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports

class CarRemoteGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("智能小车双模测控工作站")
        self.root.geometry("740x670")
        self.root.minsize(700, 620)

        self.ser = None
        self.is_connected = False
        self.rx_thread = None
        self.hb_thread = None
        self.running = True
        self.port_map = {}
        self.last_telemetry_time = 0.0

        # 车辆遥测状态
        self.telemetry = {
            "state": 0,
            "light": 0,
            "speed_pct": 50,
            "speed": 0.0,
            "distance": 0.0,
            "yaw": 0.0,
            "acc": 0.0,
            "last_rx_time": 0.0
        }

        self.create_widgets()
        self.bind_events()
        self.refresh_ports()

        # 启动后台心跳线程与界面状态定时检测
        self.hb_thread = threading.Thread(target=self.hb_worker, daemon=True)
        self.hb_thread.start()
        self.root.after(200, self.check_link_status)

    def create_widgets(self):
        style = ttk.Style()
        style.theme_use('clam')

        # 1. 串口配置面板
        conn_frame = ttk.LabelFrame(self.root, text=" 串口通信设置 (无线 DAP / USB-TTL / 蓝牙 / MSP430) ", padding=10)
        conn_frame.pack(fill="x", padx=12, pady=6)

        ttk.Label(conn_frame, text="串口设备:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.port_combo = ttk.Combobox(conn_frame, width=28, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5, pady=5, sticky="ew")

        ttk.Label(conn_frame, text="波特率:").grid(row=0, column=2, padx=5, pady=5, sticky="w")
        self.baud_combo = ttk.Combobox(conn_frame, width=10, state="readonly", values=["115200", "9600", "57600"])
        self.baud_combo.current(0)
        self.baud_combo.grid(row=0, column=3, padx=5, pady=5)

        self.btn_connect = ttk.Button(conn_frame, text="打开连接", command=self.toggle_connection)
        self.btn_connect.grid(row=0, column=4, padx=10, pady=5)

        self.btn_refresh = ttk.Button(conn_frame, text="刷新端口", command=self.refresh_ports)
        self.btn_refresh.grid(row=0, column=5, padx=5, pady=5)

        # 链路精准状态指示
        self.lbl_link_status = tk.Label(conn_frame, text="⚪ 串口未打开 (等待用户连接)",
                                        font=("Segoe UI", 10, "bold"), fg="#757575", anchor="w")
        self.lbl_link_status.grid(row=1, column=0, columnspan=6, padx=5, pady=4, sticky="w")

        # 2. 遥测状态仪表盘
        dash_frame = ttk.LabelFrame(self.root, text=" 车载仪表盘状态 (来自 G3507 & JY61P 陀螺仪) ", padding=10)
        dash_frame.pack(fill="x", padx=12, pady=6)

        # 速度与里程
        self.lbl_speed = tk.Label(dash_frame, text="车速: 0.00 m/s", font=("Segoe UI", 16, "bold"), fg="#1E88E5")
        self.lbl_speed.grid(row=0, column=0, padx=16, pady=4, sticky="w")

        self.lbl_dist = tk.Label(dash_frame, text="里程: 0.00 m", font=("Segoe UI", 16, "bold"), fg="#43A047")
        self.lbl_dist.grid(row=0, column=1, padx=16, pady=4, sticky="w")

        # 陀螺仪航向角与总加速度
        self.lbl_yaw = tk.Label(dash_frame, text="航向角(Yaw): +000.0°", font=("Segoe UI", 15, "bold"), fg="#8E24AA")
        self.lbl_yaw.grid(row=1, column=0, padx=16, pady=4, sticky="w")

        self.lbl_acc = tk.Label(dash_frame, text="净加速度: 0.00 g", font=("Segoe UI", 15, "bold"), fg="#D81B60")
        self.lbl_acc.grid(row=1, column=1, padx=16, pady=4, sticky="w")

        # 运行模式与限速
        self.lbl_mode = tk.Label(dash_frame, text="导航模式: [ 遥控模式 ]", font=("Segoe UI", 14, "bold"), fg="#1E88E5")
        self.lbl_mode.grid(row=2, column=0, padx=16, pady=4, sticky="w")

        self.lbl_limit = tk.Label(dash_frame, text="当前生效限速: 50%", font=("Segoe UI", 14, "bold"), fg="#FB8C00")
        self.lbl_limit.grid(row=2, column=1, padx=16, pady=4, sticky="w")

        # 状态字
        self.lbl_status = tk.Label(dash_frame, text="工况: STANDBY  |  MSP430 屏幕联动: 待机", font=("Consolas", 10), fg="#555")
        self.lbl_status.grid(row=3, column=0, columnspan=2, padx=16, pady=4, sticky="w")

        # 3. 动态调速滑块区
        speed_frame = ttk.LabelFrame(self.root, text=" 动力调速滑块 (双向同步, 支持 +/- 微调) ", padding=10)
        speed_frame.pack(fill="x", padx=12, pady=6)

        slider_box = ttk.Frame(speed_frame)
        slider_box.pack(fill="x", padx=10, pady=4)

        ttk.Label(slider_box, text="动力限速:").pack(side="left", padx=5)
        self.lbl_slider_val = ttk.Label(slider_box, text="50%", font=("Segoe UI", 11, "bold"), width=6)
        self.speed_slider = ttk.Scale(slider_box, from_=0, to=100, orient="horizontal", command=self.on_slider_change)
        self.speed_slider.set(50)
        self.speed_slider.pack(side="left", fill="x", expand=True, padx=10)
        self.lbl_slider_val.pack(side="left", padx=5)

        ttk.Button(slider_box, text="减速 (-5%)", command=lambda: self.adjust_speed(-5)).pack(side="left", padx=3)
        ttk.Button(slider_box, text="加速 (+5%)", command=lambda: self.adjust_speed(5)).pack(side="left", padx=3)

        # 4. 运动遥控操作区
        ctrl_frame = ttk.LabelFrame(self.root, text=" 运动遥控台 (支持键盘全局快捷键) ", padding=10)
        ctrl_frame.pack(fill="both", expand=True, padx=12, pady=6)

        grid_frame = ttk.Frame(ctrl_frame)
        grid_frame.pack(pady=4)

        # 方向按键矩阵
        btn_w = ttk.Button(grid_frame, text="前进 (W)", width=14, command=lambda: self.send_cmd('w'))
        btn_w.grid(row=0, column=1, pady=4)

        btn_a = ttk.Button(grid_frame, text="左转 (A)", width=14, command=lambda: self.send_cmd('a'))
        btn_a.grid(row=1, column=0, padx=6)

        btn_stop = tk.Button(grid_frame, text="刹车 (空格)", width=14, bg="#FFCDD2", fg="#B71C1C", font=("Segoe UI", 10, "bold"),
                             command=lambda: self.send_cmd(' '))
        btn_stop.grid(row=1, column=1, padx=6)

        btn_d = ttk.Button(grid_frame, text="右转 (D)", width=14, command=lambda: self.send_cmd('d'))
        btn_d.grid(row=1, column=2, padx=6)

        btn_s = ttk.Button(grid_frame, text="后退 (S)", width=14, command=lambda: self.send_cmd('s'))
        btn_s.grid(row=2, column=1, pady=4)

        # 辅助功能区
        sub_frame = ttk.Frame(ctrl_frame)
        sub_frame.pack(pady=4)

        ttk.Button(sub_frame, text="启停切换 (1)", width=12, command=lambda: self.send_cmd('1')).pack(side="left", padx=4)
        ttk.Button(sub_frame, text="里程清零 (2)", width=12, command=lambda: self.send_cmd('2')).pack(side="left", padx=4)
        ttk.Button(sub_frame, text="红绿灯切换 (3)", width=14, command=lambda: self.send_cmd('3')).pack(side="left", padx=4)

        # 底部提示
        tips = ttk.Label(self.root, text="💡 提示：点击窗口后直接敲击 W/A/S/D 行进，空格制动，+/- 调速，S1/S2 与 MSP430 同步联动", font=("Segoe UI", 9))
        tips.pack(side="bottom", pady=4)

    def bind_events(self):
        keys = ['w', 's', 'a', 'd', ' ', '1', '2', '3', '+', '=', '-', '_', 'r', 'g', 'z', 'x']
        for k in keys:
            self.root.bind(f"<KeyPress-{k}>", lambda e, cmd=k: self.on_key(cmd))

    def on_key(self, key):
        if key in ['+', '=']:
            self.adjust_speed(5)
        elif key in ['-', '_']:
            self.adjust_speed(-5)
        else:
            self.send_cmd(key)

    def on_slider_change(self, val):
        pct = int(float(val))
        if hasattr(self, 'lbl_slider_val'):
            self.lbl_slider_val.config(text=f"{pct}%")
        self.send_speed_packet(pct)

    def adjust_speed(self, delta):
        cur = int(self.speed_slider.get())
        new_val = max(0, min(100, cur + delta))
        self.speed_slider.set(new_val)
        self.on_slider_change(new_val)

    def send_speed_packet(self, pct):
        if self.ser and self.ser.is_open:
            try:
                pkt = f"!SET:SPD={pct:d}#"
                self.ser.write(pkt.encode('ascii'))
            except Exception as e:
                print(f"调速发送异常: {e}")

    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        self.port_map = {}
        display_list = []
        for p in ports:
            label = f"{p.device} ({p.description})"
            self.port_map[label] = p.device
            display_list.append(label)
        self.port_combo['values'] = display_list
        if display_list:
            self.port_combo.current(0)
        else:
            self.port_combo.set("")

    def toggle_connection(self):
        if not self.is_connected:
            selected_label = self.port_combo.get()
            if not selected_label:
                messagebox.showwarning("提示", "未选择任何有效串口，请先插入无线调试器/串口线并刷新！")
                return
            port = self.port_map.get(selected_label, selected_label)
            baud = int(self.baud_combo.get())
            try:
                self.ser = serial.Serial(port, baud, timeout=0.1)
                self.is_connected = True
                self.last_telemetry_time = 0.0
                self.btn_connect.config(text="断开连接")
                self.port_combo.config(state="disabled")
                self.baud_combo.config(state="disabled")

                self.rx_thread = threading.Thread(target=self.rx_worker, daemon=True)
                self.rx_thread.start()
            except Exception as e:
                messagebox.showerror("连接错误", f"无法打开串口 {port}: {e}")
        else:
            self.is_connected = False
            if self.ser:
                try: self.ser.close()
                except: pass
            self.btn_connect.config(text="打开连接")
            self.port_combo.config(state="readonly")
            self.baud_combo.config(state="readonly")

    def send_cmd(self, char):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(char.encode('ascii'))
            except Exception as e:
                print(f"发送异常: {e}")

    def hb_worker(self):
        """每 200ms 向小车下发一次 'h' 心跳，维持小车绿灯常亮与在线指示"""
        while self.running:
            if self.is_connected and self.ser and self.ser.is_open:
                try:
                    self.ser.write(b'h')
                except Exception:
                    pass
            time.sleep(0.2)

    def check_link_status(self):
        """精准检测串口与车载遥测链路状态"""
        if not self.is_connected:
            self.lbl_link_status.config(text="⚪ 串口未打开 (等待用户连接)", fg="#757575")
        else:
            now = time.time()
            dt = now - self.last_telemetry_time
            if self.last_telemetry_time > 0 and dt < 1.0:
                self.lbl_link_status.config(
                    text=f"🟢 双向链路正常 (小车在线 20Hz, 遥测延迟: {int(dt * 1000)}ms)",
                    fg="#2E7D32"
                )
            else:
                self.lbl_link_status.config(
                    text="🔴 串口已打开，但等待小车应答超时 (未收到20字节遥测，请检查电源/波特率/接线)",
                    fg="#C62828"
                )
        self.root.after(200, self.check_link_status)

    def rx_worker(self):
        buf = bytearray()
        while self.is_connected and self.ser and self.ser.is_open:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    buf.extend(data)
                    # 尝试解析 20 字节二进制协议帧 (0xAA 0x55 ... 0x55 0xAA)
                    while len(buf) >= 20:
                        if buf[0] == 0xAA and buf[1] == 0x55:
                            if buf[18] == 0x55 and buf[19] == 0xAA:
                                # 校验和验证
                                chk = sum(buf[2:17]) & 0xFF
                                if chk == buf[17]:
                                    speed_pct = buf[2]
                                    car_state = buf[3]
                                    light_state = buf[4]
                                    spd_mm = struct.unpack(">h", buf[5:7])[0]
                                    dist_mm = struct.unpack(">i", buf[7:11])[0]
                                    yaw_01 = struct.unpack(">h", buf[11:13])[0]
                                    acc_001g = struct.unpack(">h", buf[13:15])[0]

                                    parsed = {
                                        "speed_pct": speed_pct,
                                        "state": car_state,
                                        "light": light_state,
                                        "spd": spd_mm / 1000.0,
                                        "dist": dist_mm / 1000.0,
                                        "yaw": yaw_01 / 10.0,
                                        "acc": acc_001g / 100.0
                                    }
                                    self.last_telemetry_time = time.time()
                                    self.root.after(0, self.update_telemetry, parsed)
                                    buf = buf[20:]
                                    continue
                            # 帧尾不对，滑动窗口
                            buf = buf[1:]
                        else:
                            # 查找 0xAA 帧头
                            idx = buf.find(0xAA)
                            if idx > 0:
                                buf = buf[idx:]
                            elif idx == -1:
                                buf.clear()
                            else:
                                break
            except Exception:
                pass
            time.sleep(0.005)

    def update_telemetry(self, data):
        if "spd" in data:
            self.lbl_speed.config(text=f"车速: {data['spd']:.2f} m/s")
        if "dist" in data:
            self.lbl_dist.config(text=f"里程: {data['dist']:.2f} m")
        if "yaw" in data:
            self.lbl_yaw.config(text=f"航向角(Yaw): {data['yaw']:+06.1f}°")
        if "acc" in data:
            self.lbl_acc.config(text=f"净加速度: {data['acc']:.2f} g")
        if "speed_pct" in data:
            self.lbl_limit.config(text=f"当前生效限速: {data['speed_pct']}%")
        state_code = data.get("state", 0)
        mode_str = "循线模式 (TRACK)" if state_code == 2 else "遥控模式 (MANUAL)"
        mode_color = "#00ACC1" if state_code == 2 else "#1E88E5"
        self.lbl_mode.config(text=f"导航模式: [ {mode_str} ]", fg=mode_color)

        state_str = "TRACK (循线)" if state_code == 2 else ("RUN (运行)" if state_code == 1 else "STOP (停止)")
        self.lbl_status.config(text=f"运行工况: {state_str}  |  MSP430 屏幕联动: 同步正常")

if __name__ == "__main__":
    root = tk.Tk()
    app = CarRemoteGUI(root)
    root.mainloop()
