# bsp/oled 接口说明

## 模块职责

`bsp/oled` 负责 SSD1306 OLED 的初始化、清屏、清行、字符、字符串、数字和位图显示。当前实现使用 SysConfig 配置的硬件 I2C 控制器，并依赖 `bsp/time` 提供初始化阶段所需的毫秒阻塞延时。

该模块不负责菜单布局、错误文案组织、页面刷新策略或业务状态显示。

## 硬件映射

I2C 外设实例、总线速率和 PA10/PA11 引脚复用由 SysConfig 生成文件提供。当前生成配置中存在：

```c
#define OLED_INST         I2C0
#define OLED_BUS_SPEED_HZ 400000
#define GPIO_OLED_SDA_PIN DL_GPIO_PIN_10
#define GPIO_OLED_SCL_PIN DL_GPIO_PIN_11
```

SSD1306 的 7 位 I2C 地址在驱动内部固定为 `0x3C`。旧软件 I2C 中使用的 `0x78` 是包含写方向位的 8 位地址，不再作为硬件 I2C 传输地址使用。

## 屏幕尺寸宏

```c
#define OLED_WIDTH      128U
#define OLED_HEIGHT     64U
#define OLED_PAGE_COUNT 8U
```

坐标约定：

- `x` 为列坐标，范围 `0..127`。
- `page` 为页坐标，范围 `0..7`。
- 8 点高字体占 1 页，16 点高字体占 2 页。

## 公开接口

### `void OLED_Init(void)`

初始化 SSD1306，并清屏后打开显示。

### `void OLED_DisplayOn(void)`

打开 OLED 显示。

### `void OLED_DisplayOff(void)`

关闭 OLED 显示。

### `void OLED_Display_On(void)`

兼容旧命名，内部调用 `OLED_DisplayOn()`。

### `void OLED_Display_Off(void)`

兼容旧命名，内部调用 `OLED_DisplayOff()`。

### `void OLED_ColorTurn(uint8_t enable)`

设置正常显示或反色显示。`enable == 0` 为正常显示，`enable == 1` 为反色显示。

### `void OLED_Clear(void)`

清空全屏。

### `void OLED_ClearLine(uint8_t page)`

清空指定页。`page >= OLED_PAGE_COUNT` 时直接返回。

### `void OLED_ShowChar(uint8_t x, uint8_t page, uint8_t chr, uint8_t sizey)`

显示一个 ASCII 字符。当前支持 `sizey == 8` 和 `sizey == 16`。不可显示字符会替换为 `?`。

### `void OLED_ShowNum(uint8_t x, uint8_t page, uint32_t num, uint8_t len, uint8_t sizey)`

按指定宽度显示无符号整数。

### `void OLED_ShowString(uint8_t x, uint8_t page, const char *chr, uint8_t sizey)`

显示以 `\0` 结尾的字符串。超过屏幕宽度的内容会截断。

### `void OLED_ShowStringClearLine(uint8_t x, uint8_t page, const char *chr, uint8_t sizey)`

先清空指定页，再显示字符串。

### `void OLED_DrawBMP(uint8_t x, uint8_t page, uint8_t sizex, uint8_t sizey, const uint8_t *bmp)`

按页格式绘制位图。`sizey` 按 8 点一页向上取整。

### `void OLED_DrawCalibrationScreen(uint8_t step)`

画一屏巡线校准向导提示，参数取 `OLED_CALIB_STEP_xxx`：

| 取值 | 画面 |
| --- | --- |
| `OLED_CALIB_STEP_WHITE` (1) | 24px「循线校准」+ 16px「请放在白色上」 |
| `OLED_CALIB_STEP_BLACK` (2) | 24px「循线校准」+ 16px「请放在黑色上」 |
| `OLED_CALIB_STEP_OK` (3) | 24px「校准成功」整屏居中 |
| `OLED_CALIB_STEP_ERR` (4) | 24px「校准失败」+ 16px「请重试」 |

只有两行字，全部水平居中：标题上边距 8px、行间 8px，成功后只剩一行并垂直居中。
汉字点阵见 `bsp/oled/oled_font_cn.h`（24px / 16px，1bpp，由 `tools/gen_oled_cn_font.py`
用微软雅黑 Bold 渲染生成），不在运行时缩放，所以字库要加字需重新生成该头文件。

**退出由上层控制**：本函数只负责画，不改变任何状态。上层在成功后停留
`1800ms` 自动切回 `OLED_DrawCompassDashboard` 仪表盘。

### 屏幕缺席与总线卡死（驱动内自愈）

OLED 挂在 I2C1（PA10/PA11）上，模块自带的 4.7k 上拉是该总线**唯一**的上拉。屏幕一旦
掉电/拔掉，两根线悬空读低，`I2C1->MSR` 会一直是 `0x60`（BUSBSY | IDLE），此后每次
写屏都要等一次超时 —— 几十条事务叠起来会把主循环拖到 1Hz，遥测都发不出去。

驱动为此加了两层保护：

| 机制 | 行为 |
| --- | --- |
| `OLED_ProbeAck()` | 每 `OLED_PRESENCE_PERIOD_MS`(2s) 发一个地址字节；**必须**同时满足「起始前无 BUSBSY」「BUSY 真的置起过」「收尾无 ERROR」才算在线。只看 ERROR 会把"START 根本没发出去"误判成在线 |
| `OLED_I2C_BusRecover()` | 仅在屏在线却检测到 BUSBSY 时触发：把 SCL/SDA 抢成 GPIO，用「高阻=释放 / 输出低=拉低」补 9 个时钟 + 手工 STOP，再把复用功能和 HiZ 原样恢复 |

屏幕不在线时 `OLED_I2C_Write()` 直接返回，显存照常生成 —— 插回屏幕后 2s 内自动恢复显示。
`OLED_I2C_TIMEOUT` 也从 100000 收到 20000（单次事务约 2ms），坏总线最多只损失几十毫秒。

### `void OLED_DrawGrayscaleMonitor(const uint8_t analog[8], uint8_t digital_mask, bool online)`

八路灰度实时监视整屏（遥控任务里按 UP 键切进来）：

- 8 行 = 8 个 page，每行 `[通道号] [黑白方块] [模拟量条形图] [三位数值]`；
  条形图 16..101 共 86px，数值右对齐到 127，百位/十位为 0 时留空不画前导零。
- `digital_mask` bit0 对应探头 1，1 画实心方块（压黑），0 画空心方块（白底）。
- `online == false` 时整屏显示 `NO SENSOR` + `CHECK I2C 0x4C`，避免一排 0 被误读成全白。

## 内部接口

以下能力已收敛到 `oled.c` 内部，不再由头文件公开：

- SSD1306 写命令和写数据
- 设置页地址和列地址
- 硬件 I2C 阻塞发送、总线空闲等待和分段页数据写入
- 数字显示使用的幂函数

上层应只调用公开显示接口。
