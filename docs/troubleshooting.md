# 调试与排障手册

> 本文档记录的是**在本车上实际验证过**的排查手段，不是通用教程。
> 遇到"某个模块没数据""OLED 不刷新""遥测掉帧"时按顺序往下走，通常几分钟能定位。

---

## 0. 先确认链路是通的

| 手段 | 命令 / 现象 |
| --- | --- |
| 无线串口 | PC 上 `COM7`（VID `FAED:4870`），115200 8N1，直接收发 `!CAL:*` 与 20 字节遥测帧 |
| 调试器 | Horco CMSIS-DAP，`python -m pyocd list` 能看到 |
| 内核是否在跑 | 读 `g_bsp_time_ms`（地址查 map）两次，间隔 1s 应增加约 1000 |

**烧录后必须复位**：Keil 命令行 `-f` 烧完会把内核留在 halt 状态，
需要 `python -m pyocd reset -t cortex_m`，否则表现为"烧了没反应"。

```bash
# 读一个 RAM 变量（符号地址从 .map 的 Local Symbols 段找）
python -m pyocd commander -t cortex_m -M attach -c "read32 0x20200430"
```

---

## 1. 主循环"变慢"类问题

典型现象：遥测从 20Hz 掉到 0.5Hz、按键像失灵、OLED 不刷新。

**第一步永远是量循环节拍**，不要猜：

```bash
# 陀螺仪轮询计数，1s 内应增长几十次；不涨 = 主循环被卡住
python -m pyocd commander -t cortex_m -M attach \
  -c "read32 0x20200594" -c "sleep 1000" -c "read32 0x20200594"
```

**第二步抓 PC**，看卡在哪个函数：

```bash
python -m pyocd commander -t cortex_m -M attach \
  -c "halt" -c "reg pc" -c "reg lr" -c "go" -c "sleep 250" \
  -c "halt" -c "reg pc" -c "reg lr" -c "go"
```

把 PC 拿到反汇编里对照：

```bash
fromelf.exe -c --cpu=Cortex-M0+ Objects/NUEDC2025_MSPM0G3507.axf > dis.txt
```

> [!TIP]
> 反复采样都落在同一个 `while (...)` 轮询上，基本可以断定是**某个外设的等待循环没有出口**，
> 而不是"逻辑算错了"。

---

## 2. I2C 总线问题（最容易踩）

### 2.1 看状态寄存器，一眼定性

MSPM0 每个 I2C 实例的 `MSR`（`基址 + 0x1218`）：

| 值 | 含义 | 下一步 |
| --- | --- | --- |
| `0x20` | IDLE，总线空闲 | 正常 |
| `0x60` | **BUSBSY + IDLE** | 总线被拉低：从机把 SDA 拉住，或**根本没有从机/没有上拉** |
| `0x02` | ERR | 上一次事务 NACK 或仲裁失败 |
| `0x40` | BUSBSY | 传输进行中 |

I2C0（灰度 + 陀螺仪）基址 `0x400F0000` → MSR `0x400F1218`；
I2C1（OLED）基址 `0x400F2000` → MSR `0x400F3218`。

### 2.2 判断"是模块挂了"还是"总线被固件弄死了"

关键一招：**临时给引脚开内部上拉，再读输入电平**。

```bash
# 例：OLED 的 PA10/PA11（PINCM21/22）。先读 DIN，再改成 GPIO+内部上拉读一次
python -m pyocd commander -t cortex_m -M attach \
  -c "read32 0x400a1380" \
  -c "write32 0x40428054 0x00060081" -c "write32 0x40428058 0x00060081" -c "sleep 50" \
  -c "read32 0x400a1380" \
  -c "write32 0x40428054 0x02040088" -c "write32 0x40428058 0x02040088"
```

- 加上拉后**变高** → 没有任何器件在拉低，是**外部上拉没了**（模块没供电/没插）；
- 加上拉后**仍为低** → 有器件真的把线拉死了（模块故障或短路）；
- 事后**务必写回**原来的 PINCM 值，否则该引脚一直是 GPIO。

### 2.3 感为八路灰度：必须用重复起始

实测（100kHz）：

| 读时序 | 结果 |
| --- | --- |
| 写寄存器 → STOP → 立刻 START 读 | **失败**，RX 等不到数据 |
| 写寄存器 → STOP → 延时 2ms → START | 成功 |
| 写寄存器 → **不发 STOP** → 重复起始 | 成功且最快 |

模块内部是 STM8，STOP 后需要一段总线空闲；主机紧接着发 START 会被当成噪声。

### 2.4 写多字节：TX FIFO 只有 8 深

写阈值是「寄存器 + 8 字节」= 9 字节，而 FIFO 只有 8。
`DL_I2C_fillControllerTXFIFO()` 填满就停，**剩下的必须在传输中补**，
否则第 9 字节永远等不到 → 超时。这个坑的表现是"校准永远卡在采集黑线"。

### 2.5 共用总线的互斥

灰度（0x4C）与 JY61P（0x50）共用 I2C0。JY61P 用"写寄存器不发 STOP → 重复起始读"的复合事务，
中间被别的设备插一脚必坏。所以：

- 校准向导打开时主动 `JY61P_I2C_SetSuspended(true)`；
- 灰度驱动每次失败都 `resetControllerTransfer` + flush，并做 9 时钟总线恢复；
- 不要把 Probe 放进 100ms 级的高频循环里。

---

## 3. OLED 问题

### 3.1 不用相机也能看屏幕内容

OLED 驱动在 RAM 里维护 1024 字节显存 `s_oled_fb`（8 page × 128 列，bit0 = 页顶）。
直接 dump 出来反解成 PNG，就是**真实显示内容**：

```bash
python -m pyocd commander -t cortex_m -M attach \
  -c "halt" -c "savemem 0x202006f1 1024 C:/tmp/fb.bin" -c "go"
```

```python
from PIL import Image
d = open("fb.bin", "rb").read()
img = Image.new("L", (128, 64), 0); px = img.load()
for page in range(8):
    for x in range(128):
        b = d[page * 128 + x]
        for bit in range(8):
            if b >> bit & 1:
                px[x, page * 8 + bit] = 255
img.resize((640, 320), Image.NEAREST).save("screen.png")
```

> [!TIP]
> 地址每次编译都可能变，去 `.map` 里搜 `s_oled_fb`。
> **抓帧前先 `halt`**，否则 1024 字节还没读完画面就被下一帧改了，会看到撕裂的图。

### 3.2 拔掉屏幕会把整机拖垮

OLED 模块自带的 4.7k 上拉是 I2C1 **唯一**的上拉。屏幕一掉电：

1. PA10/PA11 悬空读低 → `I2C1->MSR = 0x60`（BUSBSY）；
2. START 发不出去，且 **`ERROR` 位不会置位**；
3. 每条写屏事务都要等满超时，几十条叠起来把主循环拖到 1Hz。

驱动现已自带保护：2s 周期 ACK 探测（同时判 BUSBSY / BUSY 是否真的置起 / ERROR）、
9 时钟 + 手工 STOP 的总线恢复、离线跳过刷屏、离线→在线补一次 `OLED_Init()`。
所以**插回屏幕 2s 内会自动恢复**，不用重新烧固件。

### 3.3 汉字字库

`bsp/oled/oled_font_cn.h` 是生成物，不要手改：

```bash
python "NUEDC_2026/[Demo] MSPM0_Car_v5/tools/gen_oled_cn_font.py"
```

用微软雅黑 Bold 渲染 24px（标题）/ 16px（提示）后二值化。加字就改脚本里的字表重跑。
源码里不出现 UTF-8 汉字字面量，避免 Keil 编码问题。

---

## 4. 无线链路与遥测

20 字节帧：`AA 55 ... CHK 55 AA`，校验 = 从第 2 字节起前 15 字节累加和。

```python
import serial, time
p = serial.Serial("COM7", 115200, timeout=0.2); p.reset_input_buffer()
b = b""; t = time.time()
while time.time() - t < 2.0:
    b += p.read(1024)
ok = i = 0
while i + 20 <= len(b):
    if b[i:i+2] == b"\xAA\x55" and b[i+18:i+20] == b"\x55\xAA" \
       and (sum(b[i+2:i+17]) & 0xFF) == b[i+17]:
        ok += 1; i += 20
    else:
        i += 1
print("20Hz 期望 40 帧，实测：", ok)
```

- 帧数明显偏少 → 回到第 1 节查主循环；
- 完全无数据 → 查无线模块供电与 `!` 指令是否被吃掉（心跳 800ms 会刹车）。

---

## 5. 网页端调试

`tools_pc_remote/web_server.py` 既托管页面也转发串口。没有实车时可用模拟数据自测 UI：

```bash
# 起一个假遥测服务，然后用无头 Chrome 截图/跑 JS
python scratch/sim_server.py           # 监听 8099
chrome --headless=new --window-size=390,844 \
       --screenshot=shot.png http://127.0.0.1:8099/mobile.html
```

页面关键状态在控制台里可直接读写（`T` 轨迹、`A` 自动驾驶、`D` 绘制），
写自动化用例比反复手点快得多，例如：

```js
A.route = [{x:0,y:0},{x:2000,y:0},{x:2000,y:2000}];
startDrive(1); A.state = 'track';
T.pts.push({x:2000, y:1995}); autoStep();   // 应触发到达
```

---

## 6. 常见现象速查

| 现象 | 最可能的原因 | 处理 |
| --- | --- | --- |
| 灰度一直 `online=0` | 模块没吃 4.5~5V / 没共地 / 地址被改 | 量 VCC-GND；用 `PROBES` 看是否扫到 `4D~4F` |
| 灰度读数全 0 但地址有 ACK | 读寄存器用了 STOP+START | 改重复起始 |
| 校准卡在"采黑线" | 写 9 字节撞 TX FIFO 8 深 | 传输中补字节 |
| OLED 不刷新、遥测掉帧 | 屏掉电导致 I2C1 BUSBSY | 插回屏幕，2s 内自愈 |
| `UP` 键无反应 | 校准向导正在运行（UP 让位） | 先 `!CAL:EXIT#` |
| 烧录后无反应 | Keil `-f` 后内核停在 halt | `pyocd reset -t cortex_m` |
| 网页连不上串口 | COM 口被其他程序占用 | 关掉串口助手 / 上位机 |
