# bsp/grayscale_sensor 接口说明

## 模块职责

`bsp/grayscale_sensor` 驱动**感为（Ganwei）八路灰度循线模块**，通过**硬件 I2C0** 读取：

- 8 路**数字量**（模块内部比较器后的黑白判定，1 字节位图）
- 8 路**模拟量**（0~255 原始灰度，用于现场标定与阈值计算）
- 模块内部**白/黑基准与回差阈值**的写入（把标定结果固化到模块 Flash）

模块与 JY61P 陀螺仪**共用 I2C0**（PA0 = SDA / A00，PA1 = SCL / A01），100kHz。

> [!IMPORTANT]
> 本模块已从"GPIO 读 8 路电平"改为**纯 I2C**。`grayscale_sensor.h` 里保留的
> `GRAYSCALE_SENSOR_x_PORT/PIN` 宏仅为兼容旧代码，当前驱动不再使用。

## 硬件与地址

| 项目 | 值 |
| --- | --- |
| 默认 7 位地址 | `0x4C`（`GW_GRAY_I2C_ADDR`） |
| 探测地址序列 | `0x4C → 0x4D → 0x4E → 0x4F`（AD0/AD1 跳线改变后的地址） |
| 供电 | **4.5 ~ 5V**，必须与主板共地 |
| 跳线 | 只插 **SDA/SCL 上拉**跳线帽；AD0 / AD1 / PULL 不插 |
| 上拉 | 模块自带，是 I2C0 总线上灰度这一路的上拉来源 |

## 寄存器表

| 宏 | 地址 | 读/写 | 含义 |
| --- | --- | --- | --- |
| `GW_GRAY_REG_PING` | `0xAA` | 读 1B | 固定返回 `0x66`，用于在线判定 |
| `GW_GRAY_REG_DIGITAL` | `0xDD` | 读 1B | 8 路数字量位图，bit0 = 探头 1，1 = 检测到黑 |
| `GW_GRAY_REG_ANALOG_BASE` | `0xB0` ~ `0xB7` | 读 8B | 8 路模拟量，0~255 |
| `GW_GRAY_REG_NORMALIZE` | `0xCF` | 写 | 归一化开关（当前未使用） |
| `GW_GRAY_REG_CALIB_BLACK` | `0xD0` | 写 8B | 黑线基准 |
| `GW_GRAY_REG_CALIB_WHITE` | `0xD1` | 写 8B | 白底基准 |

## 三条必须遵守的时序（都是实测踩出来的）

### 1. 读寄存器必须用**重复起始**，不能"STOP 后立刻 START"

模块内部是 STM8，主机在 STOP 之后紧接着发 START 会被它当成总线噪声，直接不响应。

| 读时序（2026-11-04 实测，100kHz） | 结果 |
| --- | --- |
| 写寄存器 → **STOP** → 立刻 START 读 | **失败，RX 永远等不到数据** |
| 写寄存器 → STOP → 延时 2ms → START 读 | 成功 |
| 写寄存器 → **不发 STOP** → 重复起始读 | 成功，且最快 |

驱动实现：TX 阶段 `DL_I2C_CONTROLLER_STOP_DISABLE`，RX 阶段 `START_ENABLE + STOP_ENABLE`。
**不要照抄官方例程 `BSP/IIC.c` 的写法**——它用的正是第一种。

### 2. 写多字节时注意 TX FIFO 只有 8 深

写阈值是「寄存器地址 + 8 字节」= **9 字节**，而 MSPM0 的 I2C TX FIFO 只有 8 字节深。
`DL_I2C_fillControllerTXFIFO()` 填满即停并返回实际写入数，**剩下的必须在传输过程中补**，
否则控制器等第 9 字节等到超时 → `GrayscaleSensor_ApplyCalibration()` 返回 false，
表现是校准流程永远卡在"采集黑线"。

### 3. 失败必须走总线恢复，不能只 `return false`

任何超时/错误分支统一执行：`DL_I2C_resetControllerTransfer()` + flush TX/RX FIFO；
连续失败则调用 `Gw_I2C_BusRecover()`（GPIO 模拟开漏，补 9 个时钟 + 手工 STOP）。
否则一次失败会把**共用 I2C0 的 JY61P 陀螺仪**一起拖死。

## 公开接口

| 接口 | 说明 |
| --- | --- |
| `void GrayscaleSensor_Init(void)` | 初始化状态并探测一次；探测失败不阻塞启动 |
| `bool GrayscaleSensor_Probe(void)` | 按 `0x4C~0x4F` 顺序探测：读到 `0xAA == 0x66` 即在线，并记录实际地址 |
| `bool GrayscaleSensor_IsI2cOnline(void)` | 缓存的在线标志，不产生 I2C 流量 |
| `uint8_t GrayscaleSensor_GetI2cAddr(void)` | 返回探测到的 7 位地址 |
| `void GrayscaleSensor_Read(uint8_t digital_array[8])` | 逐路数字量，`1 = 检测到线` |
| `uint8_t GrayscaleSensor_ReadMask(void)` | 8 路位图，bit0 = 通道 0；离线时每 50ms 非阻塞重连一次 |
| `uint8_t GrayscaleSensor_ReadSingle(GRAYSCALE_SENSOR_CHANNEL ch)` | 单通道，非法通道返回 0 |
| `bool GrayscaleSensor_ReadAnalog(uint8_t analog_array[8])` | 一次读回 8 路 0~255 模拟量 |
| `bool GrayscaleSensor_CalibrateSampleWhite(uint8_t out[8])` | 连采 8 次取平均，写进内部白基准 |
| `bool GrayscaleSensor_CalibrateSampleBlack(uint8_t out[8])` | 连采 8 次取平均，写进内部黑基准 |
| `bool GrayscaleSensor_ApplyCalibration(uint8_t th_w[8], uint8_t th_b[8])` | 按官方公式算回差阈值并写入 `0xD1/0xD0` |
| `void GrayscaleSensor_GetCalibrationData(uint8_t w[8], uint8_t b[8])` | 读出最近一次标定的白/黑基准 |

标定阈值沿用感为官方公式（`No_Mcu_Ganv_Grayscale_Sensor.c`）：

```c
th_white = (white * 2 + black) / 3;   /* 写 0xD1 */
th_black = (white + black * 2) / 3;   /* 写 0xD0 */
```

白黑基准相等时自动撑开 50 个码值，并保证 `black < white`。

## 通道顺序

数字量位图与数组下标一一对应，`bit0 = CHANNEL_0 = 探头 1`：

| 逻辑通道 | 位 | 说明 |
| --- | --- | --- |
| `GRAYSCALE_SENSOR_CHANNEL_0` | bit0 | 探头 1（板边一侧） |
| `GRAYSCALE_SENSOR_CHANNEL_1` | bit1 | 探头 2 |
| … | … | … |
| `GRAYSCALE_SENSOR_CHANNEL_7` | bit7 | 探头 8（另一侧板边） |

## 超时与健康

| 宏 | 默认 | 说明 |
| --- | --- | --- |
| `GW_I2C_TIMEOUT_CYCLES` | 4000 | 轮询次数，约 1ms；I2C 单字节只要几十微秒，已极宽松 |
| 在线重连节流 | 50ms | `ReadMask()` 离线时的非阻塞重试间隔 |

> [!NOTE]
> 旧版把超时写成 200000 次循环（≈ 60~90ms），一次 Probe 扫 4 个地址最坏能把主循环
> 按住几百毫秒，进而掐断 20Hz 遥测。**不要把超时改大**。

## 与上层的关系

- `middleware/line_follow` 只消费 `GrayscaleSensor_ReadMask()` 的结果，不关心 I2C 细节。
- `app/app_remote_car_task.c` 的巡线校准向导负责"白底 → 黑线"两拍采样，并调用
  `ApplyCalibration()` 落盘；OLED 提示与状态机见 `bsp_oled.md`。
- 遥控模式下**平时不读灰度**，只有进入 `TRACK` 状态或打开校准向导时才访问总线；
  UP 键的"八路监视页"是第三个访问者。

## 兼容接口

旧代码里的 `TrackingSensor_Read(digital_array)` 宏仍可用，但语义已改为"经 I2C 读数字量"。
新代码请直接使用 `GrayscaleSensor_*` 接口。
