/*******************************************************************************
 * @file    main.c
 * @brief   MSP430F5529 智能车人机交互终端（显示 + 按键 + 旋钮）
 *          （课程设计：基于 MSP430F5529 的多模态无线交互与惯导融合智能小车）
 * @author  王佳宁 (U202413712), 电信创新2402班
 *
 * ── 拓扑（2026-11 按实车接线修正）──────────────────────────────────────
 *   全部外设都在天猛星 MSPM0G3507 上：
 *     JY61P  -> I2C0  PA0(SDA)/PA1(SCL)
 *     无线串口 -> UART1 PA8(TX)/PA9(RX)      -> PC 上位机
 *     MSP430   -> UART0 PB0(TX)/PB1(RX)      -> 本板 USCI_A1 (P4.4/P4.5)
 *     OLED     -> I2C1 PA10/PA11
 *
 *   因此本板只保留 **一个** 串口 UCA1，与天猛星 UART0 直连：
 *     - 收：天猛星 20Hz 的 20 字节遥测帧 -> 本地 LCD 显示
 *     - 发：按键 / 旋钮 / 心跳 -> 天猛星解析并执行
 *
 *   旧的"双串口中继"架构在这块板上物理不成立：P3.3/P3.4 是板载 CMA3000
 *   加速度计的 ACCEL_SIMO/SOMI，没有引出到任何排针，再驱动 UCA0 只会
 *   把模拟传感器总线当串口用（还让 RX 悬空收噪声）。已彻底删除。
 ******************************************************************************/
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

#include "HAL_Board.h"
#include "HAL_Dogs102x6.h"
#include "HAL_PMM.h"
#include "HAL_UCS.h"

#define TX_BUF_SIZE     128U    /* 发送环形缓冲 (uint8_t 自动回绕) */
#define TX_BUF_MASK     (TX_BUF_SIZE - 1U)  /* 必须按 2 的幂取模！ */
#define FRAME_LEN       20U
#define HB_PERIOD_MS    500U    /* 2Hz 心跳，告诉天猛星"显示终端还在" */
#define KEY_DEBOUNCE_MS 20U     /* 按键消抖窗口：连续 20ms 电平稳定才认可 */
#define KEY_COUNT       2U      /* S1(P1.7) / S2(P2.2) */
#define ADC_TIMEOUT     20000U
#define LINK_TIMEOUT_MS 600U    /* 超过这个时间没收到遥测帧 = 链路断开 */
#define LINK_RESYNC_MS  1500U   /* 超过这个时间没收到帧 = 软复位 UCA1 重同步 */

/* 遥测数据包（天猛星 G3507 下发的 20 字节定长帧） */
typedef struct {
    uint8_t state;        /* 0=STOP, 1=RUN, 3=BRAKE（帧里 [3] 的低 7 位） */
    uint8_t light;        /* 0=RED, 1=GREEN */
    uint8_t pc_link;      /* 1 = 天猛星已连上位机（帧里 [3] 的最高位） */
    int16_t speed_mm_s;   /* 线速度 (mm/s)，来自霍尔编码器 */
    int32_t dist_mm;      /* 累计行驶里程 (mm)，来自霍尔编码器 */
    int16_t yaw_01;       /* 偏航角 (0.1 deg)，来自 JY61P */
    int16_t acc_001g;     /* 前向加速度 (0.01 g)，来自 JY61P X 轴 */
    int8_t  pwm_l;
    int8_t  pwm_r;
    uint32_t last_rx_tick;
} VehicleTelemetry;

/* 必须 volatile：只有接收中断会写它，主循环 5Hz 读它做显示。
 * 不加 volatile 时编译器可以把整个结构体缓存到寄存器里，
 * 表现就是"数据其实在更新，屏幕却一直停在某一帧"。 */
static volatile VehicleTelemetry g_telemetry;

/* ---- 接收状态机 ---- */
static volatile uint8_t  s_rx_buf[FRAME_LEN];
static volatile uint8_t  s_rx_idx;
static volatile uint32_t s_last_byte_tick;

/* ---- 发送环形缓冲 ---- */
static volatile uint8_t s_tx_buf[TX_BUF_SIZE];
static volatile uint8_t s_tx_head;
static volatile uint8_t s_tx_tail;

static volatile uint32_t s_system_ticks;

/* 按键事件：bit0 = S1(P1.7)，bit1 = S2(P2.2)。
 * 只在"确认按下"的那一次跳变置位，长按不会重复触发。 */
static volatile uint8_t  s_key_event;

/* 消抖状态机：s_key_stable 的某位为 1 表示该键当前处于"松开"（上拉高电平）。
 * 这两个变量只在 1ms 定时中断里读写，主循环不碰，天然无竞争。 */
static uint8_t s_key_stable = 0x03U;
static uint8_t s_key_cnt[KEY_COUNT];

/* ---- 诊断计数：调试时可直接从芯片内存读出，不用示波器 ----
 * 用 nm430 -a 查地址，再用 DSLite memory 读取即可判断串口是否真的在工作。 */
static volatile uint32_t s_dbg_tx_sent;      /* 已写入 TXBUF 的字节数 */
static volatile uint32_t s_dbg_tx_drop;      /* 环形缓冲满而丢弃的字节数 */
static volatile uint32_t s_dbg_rx_bytes;     /* 收到的字节数 */
static volatile uint32_t s_dbg_rx_err;       /* 接收错误(UCOE/UCFE/UCPE)次数 */
static volatile uint32_t s_dbg_rx_frame;     /* 校验通过的遥测帧数 */
static volatile uint32_t s_dbg_rx_recover;   /* UCA1 软复位次数（见 Uart1_HealthCheck） */
static uint32_t          s_last_recover_tick;

static volatile uint8_t g_speed_pct = 50;     /* 界面显示用（旋钮设定值） */
/* 故意初始化成 0xFF：上电后第一次 Wheel_CheckUpdate 一定与旋钮实测值不同，
 * 于是会立刻下发一次 !SET:SPD=xx#，把天猛星复位后的默认限速同步过来，
 * 同时这也是"上行链路是否活着"的第一个探针。 */
static uint8_t s_knob_pct = 0xFFU;

/* 16x16 晶晰 WiFi 连接图标 (同心圆弧信号波 + 实心基底圆点) */
static const uint8_t s_wifi_conn_icon[] = {
    16, 2,
    0x00, 0x10, 0x31, 0x23, 0x62, 0x46, 0x44, 0x44, 0x44, 0x44, 0x46, 0x62, 0x23, 0x31, 0x10, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x66, 0x4F, 0x4F, 0x66, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* 16x16 晶晰 WiFi 未连接图标 (清晰同心圆弧 + 底部醒目对称交叉叉号 'X') */
static const uint8_t s_wifi_disc_icon[] = {
    16, 2,
    0x00, 0x20, 0x40, 0x44, 0x84, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x84, 0x44, 0x40, 0x20, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x84, 0xCC, 0x78, 0x30, 0x30, 0x78, 0xCC, 0x84, 0x00, 0x00, 0x00, 0x00
};

/* ======================== 无浮点定点格式化 ========================
 * 不用 printf 浮点：省 4KB 代码、避免 >150B 的栈峰值，25MHz 下也快得多。 */
static void fmt_fixed(char *out, int32_t v, uint8_t dec, bool force_sign)
{
    char tmp[12];
    uint8_t i = 0U, k = 0U;
    uint32_t u;
    uint32_t p = 1U;

    if (v < 0) {
        out[k++] = '-';
        u = (uint32_t)(-v);
    } else {
        if (force_sign) {
            out[k++] = '+';
        }
        u = (uint32_t)v;
    }

    for (uint8_t d = 0U; d < dec; d++) {
        p *= 10U;
    }

    uint32_t ip = (dec == 0U) ? u : (u / p);
    uint32_t fp = (dec == 0U) ? 0U : (u % p);

    if (ip == 0U) {
        tmp[i++] = '0';
    }
    while (ip != 0U) {
        tmp[i++] = (char)('0' + (ip % 10U));
        ip /= 10U;
    }
    while (i != 0U) {
        out[k++] = tmp[--i];
    }

    if (dec != 0U) {
        out[k++] = '.';
        for (uint8_t d = dec; d > 0U; d--) {
            uint32_t dv = 1U;
            for (uint8_t e = 1U; e < d; e++) {
                dv *= 10U;
            }
            out[k++] = (char)('0' + (uint8_t)((fp / dv) % 10U));
        }
    }
    out[k] = '\0';
}

static void str_append(char *dst, uint8_t *pos, const char *src)
{
    while (*src != '\0') {
        dst[(*pos)++] = *src++;
    }
    dst[*pos] = '\0';
}

/* 把一行补空格到固定宽度再画。
 * 大字体每个字符固定占 7 像素列，如果新字符串比上一帧短，
 * 后面的旧字符不会被覆盖 —— 这就是 "S:2%%"（2% 后面残留了上一次 22% 的 '%'）
 * 以及里程从 4 位变 3 位后拖出残影的根本原因。补空格即可彻底消除。 */
static void pad_line(char *buf, uint8_t *pos, uint8_t width)
{
    while (*pos < width) {
        buf[(*pos)++] = ' ';
    }
    buf[*pos] = '\0';
}

/* ============================ 串口发送 ============================
 * 环形缓冲的下标必须用 TX_BUF_MASK 取模，**不能**直接用 uint8_t 自增回绕。
 * 历史缺陷：head/tail 是 uint8_t（0~255），而数组只有 128 字节，
 * 心跳每秒 8 字节，约 16 秒后 s_tx_buf[128..255] 就越界写到
 * s_rx_buf / g_telemetry / s_system_ticks 上，表现为"跑十几秒后
 * 陀螺仪数据冻结、按键失效"。现在下标恒定落在 0~127。
 *
 * 另外整段用"关中断-恢复"包住：push 侧（主循环）与 pop 侧（TX ISR）
 * 会同时改 head/tail，不是原子操作，必须互斥，否则可能丢一次 pop 或 push，
 * 导致 head/tail 关系永久错乱、环形缓冲判满后再也不发送。 */
static void Uart_Put(uint8_t byte)
{
    uint16_t sr = __get_SR_register() & GIE;
    uint8_t  next;

    __disable_interrupt();

    next = (uint8_t)((s_tx_head + 1U) & TX_BUF_MASK);
    if (next == s_tx_tail) {
        s_dbg_tx_drop++;
    } else {
        s_tx_buf[s_tx_head] = byte;
        s_tx_head = next;
    }

    __bis_SR_register(sr);      /* 恢复原来的 GIE，不要在 ISR 里误开中断 */
}

/* 非阻塞发送泵：在主循环里调用，TXBUF 空就送一个字节。
 *
 * 这里刻意**不再依赖 TX 中断**：只要 UCTXIE 因为任何原因没有被真正触发
 * （寄存器位定义、中断优先级、模块复位把它清掉……），整条上行链路会**静默失效**，
 * 现象正是"屏幕能收数据、按键和旋钮却一个字节都传不出去"，而且没有任何报错。
 * 轮询方式没有这个单点依赖：只要主循环在转，字节就一定会出去。
 * 115200 下每 87us 一个字节，主循环的节奏远快于此。 */
static void Uart_Poll_Tx(void)
{
    if (s_tx_tail == s_tx_head) {
        return;
    }
    if (UCA1IFG & UCTXIFG) {
        UCA1TXBUF = s_tx_buf[s_tx_tail];
        s_tx_tail = (uint8_t)((s_tx_tail + 1U) & TX_BUF_MASK);
        s_dbg_tx_sent++;
    }
}

static void Uart_PutBuf(const volatile uint8_t *buf, uint8_t len)
{
    for (uint8_t i = 0U; i < len; i++) {
        Uart_Put(buf[i]);
    }
}

static void Uart_PutStr(const char *str)
{
    while (*str != '\0') {
        Uart_Put((uint8_t)*str++);
    }
}

/* ============================ 底层初始化 ============================ */
static void SystemClock_Init(void)
{
    SetVCore(3);

    /* 片内 REFOCLK 32.768kHz 作为 FLL/ACLK 基准，上电瞬间起振，
     * 免除外部晶振数百毫秒起振延迟导致的 ACLK 停振与背光不亮。 */
    UCSCTL3 = SELREF__REFOCLK | FLLREFDIV__1;
    UCSCTL4 = SELA__REFOCLK | SELS__DCOCLK | SELM__DCOCLK;

    Init_FLL_Settle(25000, 762);    /* 25MHz */
}

/* UCA1: P4.4 = TXD, P4.5 = RXD —— 天猛星 UART0 (PB0/PB1) */
static void Uart1_Init(void)
{
    P4SEL |= BIT4 + BIT5;
    P4REN |= BIT5;          /* 对端未上电时 RX 不悬空 */
    P4OUT |= BIT5;

    UCA1CTL1 |= UCSWRST;
    UCA1CTL0 = 0;
    UCA1CTL1 = UCSSEL_2 + UCSWRST;      /* SMCLK 25MHz */
    UCA1BR0 = 13;                       /* 25MHz/115200: UCBR=13, UCBRF=9 */
    UCA1BR1 = 0;
    UCA1MCTL = UCOS16 | (9 << 4);
    UCA1CTL1 &= ~UCSWRST;
    UCA1IE |= UCRXIE;
}

/* 接收链路看门狗。
 *
 * MSP430 的 USCI 有个坑：UCOE(溢出) 一旦置位却没被"先读 STAT、再读 RXBUF"
 * 及时清掉，硬件会**静默停止接收** —— 后续字节全部丢弃、RXIFG 不再置位、
 * 接收中断从此再也不来。现象是"跑十几秒后屏幕数据停在最后一帧"，
 * 而程序本身仍在正常跑（心跳还发、按键还灵），极难和"对面不发"区分。
 *
 * 两道保险：
 *   1) 主循环轮询 UCA1STAT，一发现溢出立刻按标准序列清掉；
 *   2) 连续 1.5s 收不到完整帧，就软复位 UCA1 重新初始化。
 * 恢复次数记在 s_dbg_rx_recover，回读芯片即可知道到底救回来几次。
 */
static void Uart1_Recover(void)
{
    UCA1CTL1 |= UCSWRST;                /* 进入软复位，终止一切在途收发 */
    UCA1CTL0 = 0;
    UCA1CTL1 = UCSSEL_2 + UCSWRST;      /* SMCLK 25MHz */
    UCA1BR0 = 13;                       /* 25MHz/115200: UCBR=13, UCBRF=9 */
    UCA1BR1 = 0;
    UCA1MCTL = UCOS16 | (9 << 4);
    UCA1CTL1 &= ~UCSWRST;
    UCA1IE |= UCRXIE;

    s_rx_idx = 0U;                      /* 丢掉半截帧，等下一个 0xAA 重新同步 */
}

static void Uart1_HealthCheck(void)
{
    /* 1) 溢出 / 帧错 / 校验错：读 STAT 再读 RXBUF 才能清 */
    if (UCA1STAT & (UCOE | UCFE | UCPE)) {
        (void)UCA1RXBUF;
        s_rx_idx = 0U;
        s_dbg_rx_err++;
    }

    /* 2) 太久没有完整帧 -> 软复位。
     *    只在"本来收得到、后来突然收不到"时才复位：
     *    上电至今一帧都没收到，说明接线/波特率根本没对上，复位再多次也没用，
     *    只会让计数无限增长、屏幕一直在刷 *N。 */
    if (g_telemetry.last_rx_tick != 0U &&
        s_system_ticks > 3000U &&
        (s_system_ticks - g_telemetry.last_rx_tick) > LINK_RESYNC_MS &&
        (s_system_ticks - s_last_recover_tick) > LINK_RESYNC_MS) {
        s_last_recover_tick = s_system_ticks;
        s_dbg_rx_recover++;
        Uart1_Recover();
    }
}

/* ============================ 帧解析 ============================ */
static void Parse_Telemetry(const volatile uint8_t *buf)
{
    /* 注意：buf[2] 是天猛星"当前生效的限速"，只用来显示。
     * 绝对不能写回 s_knob_pct —— 否则每收到一帧（20ms）就把旋钮基准冲掉，
     * 旋钮与车速只要有一点差异，Wheel_CheckUpdate 就会每 50ms 重发一次
     * !SET:SPD=xx#，把 128 字节的发送环形缓冲灌满，按键/心跳全部发不出去。 */
    g_speed_pct = buf[2];

    g_telemetry.state      = (uint8_t)(buf[3] & 0x7FU);
    g_telemetry.pc_link    = (uint8_t)((buf[3] & 0x80U) ? 1U : 0U);
    g_telemetry.light      = buf[4];
    g_telemetry.speed_mm_s = (int16_t)(((uint16_t)buf[5] << 8) | buf[6]);
    g_telemetry.dist_mm    = (int32_t)(((uint32_t)buf[7] << 24) |
                                       ((uint32_t)buf[8] << 16) |
                                       ((uint32_t)buf[9] << 8)  |
                                       (uint32_t)buf[10]);
    g_telemetry.yaw_01     = (int16_t)(((uint16_t)buf[11] << 8) | buf[12]);
    g_telemetry.acc_001g   = (int16_t)(((uint16_t)buf[13] << 8) | buf[14]);
    g_telemetry.pwm_l      = (int8_t)buf[15];
    g_telemetry.pwm_r      = (int8_t)buf[16];
    g_telemetry.last_rx_tick = s_system_ticks;
}

static void Rx_Byte(uint8_t ch)
{
    if (s_rx_idx == 0U) {
        if (ch == 0xAAU) {
            s_rx_buf[0] = ch;
            s_rx_idx = 1U;
            s_last_byte_tick = s_system_ticks;
        }
        /* 其余字节直接丢弃：本口只接天猛星，不该出现别的内容 */
        return;
    }

    /* 帧间隔静默超时：丢弃半截帧，避免状态机被噪声卡死 */
    if ((s_system_ticks - s_last_byte_tick) > 20U) {
        s_rx_idx = 0U;
        return;
    }
    s_last_byte_tick = s_system_ticks;

    if (s_rx_idx == 1U) {
        if (ch == 0x55U) {
            s_rx_buf[1] = ch;
            s_rx_idx = 2U;
        } else if (ch == 0xAAU) {
            s_rx_buf[0] = ch;               /* 重复帧头 */
        } else {
            s_rx_idx = 0U;
        }
        return;
    }

    s_rx_buf[s_rx_idx++] = ch;
    if (s_rx_idx < FRAME_LEN) {
        return;
    }
    s_rx_idx = 0U;

    if (s_rx_buf[18] != 0x55U || s_rx_buf[19] != 0xAAU) {
        return;
    }
    uint8_t sum = 0U;
    for (uint8_t i = 2U; i <= 16U; i++) {
        sum = (uint8_t)(sum + s_rx_buf[i]);
    }
    if (sum != s_rx_buf[17]) {
        return;
    }

    s_dbg_rx_frame++;
    Parse_Telemetry(s_rx_buf);
}

/* ============================ 中断服务 ============================ */
#pragma vector = USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    switch (__even_in_range(UCA1IV, 4)) {
    case 2: {
        /* 先读 STAT 再读 RXBUF：这是清除 UCOE(溢出)/UCFE(帧错)/UCPE(校验错)
         * 的标准序列。不读 STAT 的话溢出标志会一直挂着，之后再也收不到数据。 */
        uint8_t stat = UCA1STAT;
        uint8_t ch   = UCA1RXBUF;

        if (stat & (UCOE | UCFE | UCPE)) {
            s_dbg_rx_err++;
            s_rx_idx = 0U;      /* 丢掉半截帧，等下一个 0xAA 重新同步 */
        }
        s_dbg_rx_bytes++;
        Rx_Byte(ch);
        break;
    }
    default:
        break;
    }
}

/* ===================== 按键采样 + 消抖（1ms 节拍） =====================
 *
 * 旧实现用的是端口中断：只要引脚上出现一个下降沿就进一次 ISR。
 * 机械触点按下/释放时会在 5~15ms 内弹跳好几次，于是 s_btn_pending 被
 * 反复置位，最终连着发出好几个 '1'。
 *
 * 对 S1 这种"启停切换"来说这是致命的：按一次可能先发 '1'（RUN）、
 * 再发一个 '1'（又切回 STOP），甚至发出三个 —— 现象就是"按一下没反应"
 * 或者"状态自己乱跳"，同时 S2 的清零也会被连按多次。
 *
 * 现在改成"1ms 采样 + 连续 20ms 稳定才认可"：
 *   · 采样值与当前稳定态相同  -> 计数器清零（毛刺无法累积）；
 *   · 采样值持续相反 20 次     -> 认可这次跳变；
 *   · 只有"松开 -> 按下"这一次  -> 产生事件，长按不连发。
 *
 * 这样抖动窗口内的任何跳变都会被计数清零，不需要阻塞延时，
 * 也不会再让端口中断被弹跳打成中断风暴。
 */
static void Key_Scan1ms(void)
{
    uint8_t raw = 0U;
    uint8_t evt = 0U;

    if (P1IN & BIT7) { raw |= 0x01U; }      /* 高电平 = 松开（内部上拉） */
    if (P2IN & BIT2) { raw |= 0x02U; }

    for (uint8_t i = 0U; i < KEY_COUNT; i++) {
        uint8_t m     = (uint8_t)(1U << i);
        uint8_t is_up = (uint8_t)((raw & m) ? 1U : 0U);
        uint8_t st_up = (uint8_t)((s_key_stable & m) ? 1U : 0U);

        if (is_up == st_up) {
            s_key_cnt[i] = 0U;              /* 与稳定态一致：重新计时 */
            continue;
        }

        s_key_cnt[i]++;
        if (s_key_cnt[i] >= KEY_DEBOUNCE_MS) {
            s_key_cnt[i] = 0U;
            if (is_up) {
                s_key_stable |= m;          /* 认可"松开"，不产生事件 */
            } else {
                s_key_stable &= (uint8_t)~m;    /* 认可"按下" */
                evt |= m;
            }
        }
    }

    if (evt != 0U) {
        s_key_event |= evt;
    }
}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR(void)
{
    s_system_ticks++;
    Key_Scan1ms();
}

/* ============================ 旋钮 ============================ */
static void Wheel_Init(void)
{
    P8DIR |= BIT0;
    P8OUT |= BIT0;              /* POT_PWR 供电 */
    P6SEL |= BIT5;              /* A5 */

    ADC12CTL0 = ADC12SHT0_2 + ADC12ON;
    ADC12CTL1 = ADC12SHP;
    ADC12MCTL0 = ADC12INCH_5;
    ADC12CTL0 |= ADC12ENC;
}

static uint8_t Wheel_SamplePct(void)
{
    uint32_t guard = ADC_TIMEOUT;

    ADC12CTL0 |= ADC12SC;
    while (!(ADC12IFG & BIT0)) {
        if (guard-- == 0U) {
            /* 超时返回上一次的百分比。s_knob_pct 可能被初始化成 0xFF
             * （为了上电强制下发一次），所以这里钳一下，避免把 255 显示出去。 */
            return (s_knob_pct <= 100U) ? s_knob_pct : 50U;
        }
    }
    uint16_t raw = ADC12MEM0;
    ADC12IFG &= ~BIT0;

    if (raw > 4095U) {
        raw = 4095U;
    }
    uint8_t pct = (uint8_t)(((uint32_t)raw * 100U) / 4095U);
    if (pct < 5U) {
        pct = 5U;
    }
    return pct;
}

static void Wheel_CheckUpdate(void)
{
    uint8_t cur = Wheel_SamplePct();
    int16_t diff = (int16_t)cur - (int16_t)s_knob_pct;

    if (diff >= 3 || diff <= -3) {
        s_knob_pct = cur;
        g_speed_pct = cur;

        char cmd[20];
        uint8_t p = 0U;
        char num[6];
        str_append(cmd, &p, "!SET:SPD=");
        fmt_fixed(num, (int32_t)cur, 0U, false);
        str_append(cmd, &p, num);
        str_append(cmd, &p, "#");
        Uart_PutBuf((const volatile uint8_t *)cmd, p);
    }
}

/* ============================ 按键与指示灯 ============================ */
static void Peripherals_Init(void)
{
    /* 按键：S1 = P1.7，S2 = P2.2，低电平有效（内部上拉，按下接地）。
     * 这里**不再使能端口中断**：改为 1ms 定时采样 + 20ms 稳定判定，
     * 既躲开弹跳期的中断风暴，也不用再手工清 P1IFG/P2IFG。
     * 若以后有人把 PxIE 又打开，会退回到"一个毛刺一个事件"的老毛病。 */
    P1DIR &= ~BIT7;
    P1SEL &= ~BIT7;
    P1REN |= BIT7;
    P1OUT |= BIT7;              /* 上拉 */
    P1IE  &= ~BIT7;
    P1IFG &= ~BIT7;

    P2DIR &= ~BIT2;
    P2SEL &= ~BIT2;
    P2REN |= BIT2;
    P2OUT |= BIT2;              /* 上拉 */
    P2IE  &= ~BIT2;
    P2IFG &= ~BIT2;

    P1DIR |= BIT0;      /* LED1 红 */
    P1OUT |= BIT0;
    P8DIR |= BIT1;      /* LED2 绿 */
    P8OUT &= ~BIT1;

    /* 上电瞬间就按住的键不算"按下"事件：把当前电平直接当作稳定态，
     * 必须松开、再重新按下才会触发。否则上电时手压在 S1 上，
     * 25ms 后会自动发出一帧 '1'，车莫名其妙进 RUN。 */
    s_key_stable = 0U;
    s_key_cnt[0] = 0U;
    s_key_cnt[1] = 0U;
    if (P1IN & BIT7) { s_key_stable |= 0x01U; }
    if (P2IN & BIT2) { s_key_stable |= 0x02U; }

    TA0CCTL0 = CCIE;
    TA0CCR0 = 25000U - 1U;      /* 25MHz / 1000 = 1ms */
    TA0CTL = TASSEL_2 + MC_1 + TACLR;
}

/* ============================ 仪表盘 ============================ */
/* 本口是否还在收帧（只判断"天猛星→430"这段线） */
static bool Frame_IsFresh(void)
{
    return (g_telemetry.last_rx_tick != 0U) &&
           ((s_system_ticks - g_telemetry.last_rx_tick) < LINK_TIMEOUT_MS);
}

/* 链路状态 = 本口在收帧（天猛星活着、线没断）
 *         且 帧里的 PC 在线位为 1（上位机无线也连着）
 * 这样屏幕的 LINK/DISC 与天猛星的三色灯永远一致：
 *   黄灯亮 <=> DISC，绿灯/红灯亮 <=> LINK。
 * 另外单列一个 NO RX：本口连帧都收不到，是 430 自己这段线的问题，
 * 不能和"上位机没开"混为一谈。 */
static bool Link_IsUp(void)
{
    if (!Frame_IsFresh()) {
        return false;
    }
    return (g_telemetry.pc_link != 0U);
}

/* 四舍五入的整数除法（远离零方向），避免 12.36m 被截断成 12.3m */
static int32_t div_round(int32_t v, int32_t d)
{
    if (v >= 0) {
        return (v + d / 2) / d;
    }
    return (v - d / 2) / d;
}

/* 把数字右对齐到固定宽度（不足补前导空格）。
 * 只为"字段起点不随位数变化"服务：航向只有 1~3 位，
 * 不补齐的话 " A:" 会左右晃，A 的数值列也就跟着晃。 */
static void str_right_align(char *dst, uint8_t *pos, const char *src, uint8_t width)
{
    uint8_t len = 0U;

    while (src[len] != '\0') {
        len++;
    }
    while (len < width) {
        dst[(*pos)++] = ' ';
        len++;
    }
    str_append(dst, pos, src);
}

/* 仪表盘排版（大字库每字符 7 像素列，从 x=2 起最多 14 个字符）。
 *
 * 列位是**固定**的，A 与 D 的标签和数值都严格对齐（数值同在第 9 列）：
 *
 *   列位(0起)  0  1  2  3  4  5  6  7  8  9 10 11 12 13
 *   y=2        Y  :  y2 y1 y0 sp sp  A  :  0  .  1  2  g
 *   y=26       V  :  v3 v2 v1 v0 sp  D  :  1  2  .  3  m
 *                 ^ 航向 3 位右对齐      ^ 数值固定从第 9 列开始
 *
 * 为了"数值再变也不动版式"，三处数字都被约束成定宽：
 *   · 航向 0~359        -> 3 位右对齐（"  5" / " 45" / "359"）
 *   · 加速度 0.00~3.00  -> 恒 4 字符（取绝对值，不再出现 + / - 号）
 *   · 速度   0.00~3.99  -> 恒 4 字符（取绝对值，倒车也显示大小）
 *   · 里程 0.0~99.9 / 100~9999 -> 3~4 字符（取绝对值，里程表不倒退）
 * 只有第 3 行的 "S:xx%" 仍会占 4~5 列，但它右侧的状态文字是补空格对齐的，
 * 与上面两行无关。
 *
 *   y=50  S:50% LINK       限速调节 + 与上位机通信状态（右侧 x=86 是链路图标）
 *
 * 每行都补空格到固定宽度：大字体按 7 像素栅格整字符覆盖，字符串变短时
 * 不补空格就会残留上一帧的尾巴（例如 22% -> 2% 会显示成 "2%%"）。 */
static void Update_Dashboard(void)
{
    char line_buf[24];
    char num[16];
    uint8_t p;

    /* 加速度：先取绝对值，再做死区。
     * 屏上只表达"多大"，符号没有信息量，却会让字段宽度在
     * +0.12 / -0.12 / 0.00 之间变化 —— A 的数值列就会左右跳。
     * 死区仍然保留：|a| <= 0.01g 直接按 0.00 显示。 */
    int32_t acc = (int32_t)g_telemetry.acc_001g;
    if (acc < 0)   { acc = -acc; }
    if (acc <= 1)  { acc = 0; }         /* 0.01g 以内归零 */
    if (acc > 300) { acc = 300; }       /* 限幅 3.00g，保证恒为 4 字符 */

    /* ── 行 0~1：转向角 + 加速度 ── */
    p = 0U;
    str_append(line_buf, &p, "Y:");
    {
        /* 0.1° -> 整数度，与天猛星同步保持 -180 ~ +180 范围 */
        int32_t y = div_round(g_telemetry.yaw_01, 10);
        while (y > 180)  { y -= 360; }
        while (y < -180) { y += 360; }
        fmt_fixed(num, y, 0U, false);
    }
    str_right_align(line_buf, &p, num, 4U);     /* 4 位定宽(-180~180)，与第2行V列宽对齐 */
    str_append(line_buf, &p, " A:");            /* A: 恒定在第 7 列，与 D: 纵向对齐 */
    fmt_fixed(num, acc, 2U, false);             /* 恒 4 字符，数值从第 9 列开始 */
    str_append(line_buf, &p, num);
    str_append(line_buf, &p, "g");
    pad_line(line_buf, &p, 14U);
    Dogs102x6_stringDrawBig(0, 2, line_buf, DOGS102x6_DRAW_NORMAL);

    /* ── 行 3~4：线速度 m/s（编码器）+ 里程 m ──
     * 速度取绝对值后位数恒定（0.00~3.99 都是 4 字符），
     * 于是 " D:" 和 D 的数值永远停在固定列，不会因为倒车多出一个负号。
     * 里程同样取绝对值：里程表不倒退，也就不可能撑爆 14 列。 */
    int32_t spd_mm = g_telemetry.speed_mm_s;
    if (spd_mm < 0)    { spd_mm = -spd_mm; }
    if (spd_mm > 3999) { spd_mm = 3999; }       /* 表显上限 3.99 m/s */

    p = 0U;
    str_append(line_buf, &p, "V:");
    fmt_fixed(num, div_round(spd_mm, 10), 2U, false);   /* mm/s -> m/s，恒 4 字符 */
    str_append(line_buf, &p, num);
    str_append(line_buf, &p, " D:");
    {
        /* 先按 0.1m 算，四舍五入后到 100.0（5 个字符）就改用整米，
         * 否则 99.96m 会被舍成 "100.0" 而多占一格、把整行顶出屏幕。 */
        int32_t dist = g_telemetry.dist_mm;
        uint8_t dec = 1U;
        int32_t d;

        if (dist < 0) { dist = -dist; }                 /* 只显示绝对值 */
        d = div_round(dist, 100);
        if (d >= 1000) {
            dec = 0U;
            d = div_round(dist, 1000);
        }
        if (d > 9999) { d = 9999; }                     /* 表显上限 9999 m */
        fmt_fixed(num, d, dec, false);                  /* mm -> m */
    }
    str_append(line_buf, &p, num);
    str_append(line_buf, &p, "m");
    pad_line(line_buf, &p, 14U);
    Dogs102x6_stringDrawBig(3, 2, line_buf, DOGS102x6_DRAW_NORMAL);

    /* ── 行 6~7：限速 S + 与上位机通信状态（链路图标在右侧 x=86）── */
    p = 0U;
    str_append(line_buf, &p, "S:");
    fmt_fixed(num, (int32_t)g_speed_pct, 0U, false);
    str_append(line_buf, &p, num);
    str_append(line_buf, &p, "%");
    pad_line(line_buf, &p, 7U);         /* 对齐到第 8 列，状态文字从这里开始 */
    /* 状态显示：
     *   NO RX = 430 这段线一个帧都收不到（查 P4.4/P4.5、波特率）
     *   TRK   = 小车正处于基于感为八路灰度的自主循线模式 (state == 2)
     *   DISC  = 帧正常，但上位机没连（此时天猛星亮黄灯）
     *   LINK  = 帧正常且上位机在线（天猛星亮红灯或绿灯） */
    if (!Frame_IsFresh()) {
        str_append(line_buf, &p, "NO RX");
    } else if (g_telemetry.state == 2U) {
        str_append(line_buf, &p, "TRK");
    } else {
        str_append(line_buf, &p, Link_IsUp() ? "LINK" : "DISC");
    }

    /* 格式化整行补足 12 字符（列 2~85），确保文字与图标之间有 7px 纯净空白 */
    pad_line(line_buf, &p, 12U);
    Dogs102x6_stringDrawBig(6, 2, line_buf, DOGS102x6_DRAW_NORMAL);

    /* 绘制高质感 WiFi 连接 / 未连接状态图标 (x=86..101) */
    if (Link_IsUp()) {
        Dogs102x6_imageDraw(s_wifi_conn_icon, 6, 86);
    } else {
        Dogs102x6_imageDraw(s_wifi_disc_icon, 6, 86);
    }
}

/* ============================ 上电自检配置 ============================
 * LCD 控制器本身的初始化只需要约 5ms（复位低 1ms + 释放 1ms + 几十条命令）。
 * 上电那几秒全部来自下面这套"诊断脚手架"，不是屏慢，也不是时钟慢。
 *
 *   BOOT_DIAG_FULL = 0  正常启动：约 0.3 秒直接进仪表盘（出货/日常使用）
 *   BOOT_DIAG_FULL = 1  诊断启动：约 5 秒，含 8 灯长闪、背光闪 3 下、
 *                       满屏棋盘格、绿灯 1/2/3/4 分步闪码
 *
 * 只有"屏又黑了 / 怀疑程序没跑"的时候才临时改成 1，烧一次看现象，
 * 定位完再改回 0。这样既不拖慢每次上电，也不丢掉排查手段。 */
#define BOOT_DIAG_FULL      0

/* 正常启动时 "LCD OK" 停留时间(ms)。0 = 不显示，直接进仪表盘。 */
#define BOOT_SPLASH_MS      250

/* ============================ 上电自检指示灯 ============================
 * 仅诊断模式有效。绿闪次数标记初始化进度：
 *   绿闪 1 次 = 25MHz 时钟就绪
 *   绿闪 2 次 = LCD 初始化 + 背光 + 自检图案完成
 *   绿闪 3 次 = UCA1 串口就绪
 *   绿闪 4 次 = 按键 + 定时器 + ADC 就绪，进入主循环
 * 主循环中：绿灯常亮 = 正在收遥测帧；红灯常亮 = 与天猛星链路断开。      */
static void Boot_Mark(uint8_t times)
{
#if BOOT_DIAG_FULL
    for (uint8_t i = 0; i < times; i++) {
        P8OUT |= BIT1;
        __delay_cycles(3000000);    /* 120ms @25MHz */
        P8OUT &= ~BIT1;
        __delay_cycles(3000000);
    }
#else
    (void)times;                    /* 正常启动不做分步指示 */
#endif
}

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;

    /* ═══ 第 0 步：上电指示 ═══════════════════════════════════════════
     * 诊断模式(BOOT_DIAG_FULL=1)：8 颗 LED 常亮 0.8s + 慢闪 5 次，共约 2.8s。
     *   看不到 -> 芯片没在执行这份镜像（烧录/供电/被调试器按住），
     *             此时改任何代码都不会有肉眼可见的变化。
     * 正常模式：8 颗 LED 齐亮 60ms 后熄灭，只作为"已上电"的一次确认，
     *   不拖慢启动。 */
    P1DIR |= 0x3F;              /* P1.0 ~ P1.5 = LED1,4,5,6,7,8 */
    P8DIR |= 0x06;              /* P8.1, P8.2 = LED2, LED3     */

#if BOOT_DIAG_FULL
    P1OUT |= 0x3F;
    P8OUT |= 0x06;
    __delay_cycles(800000);         /* ~800ms @ 默认 DCO 1MHz */
    P1OUT &= ~0x3F;
    P8OUT &= ~0x06;
    __delay_cycles(300000);

    for (uint8_t i = 0; i < 5; i++) {
        P1OUT |= 0x3F;
        P8OUT |= 0x06;
        __delay_cycles(200000);     /* ~200ms 亮 */
        P1OUT &= ~0x3F;
        P8OUT &= ~0x06;
        __delay_cycles(200000);     /* ~200ms 灭 */
    }
#else
    P1OUT |= 0x3F;
    P8OUT |= 0x06;
    __delay_cycles(60000);          /* ~60ms @ 默认 DCO 1MHz */
    P1OUT &= ~0x3F;
    P8OUT &= ~0x06;
#endif

    Board_init();

    /* Board_init() 会把 8 颗灯全部清零，这里重新点亮 LED1 作为"已运行"标志 */
    P1OUT |= BIT0;

    /* 板载 CMA3000 与 P3.3/P3.4 复用，本设计不用 UCA0，
     * 仍显式取消片选并断电，避免它在 P3.3/P3.4 上乱驱动。 */
    P3OUT |= BIT5;      /* ACCEL_CS = 1，取消片选 */
    P3OUT &= ~BIT6;     /* ACCEL_PWR = 0，保持断电 */

    SystemClock_Init();
    Boot_Mark(1);               /* 绿闪 1 次 = 25MHz 就绪 */

    /* ═══ 第二步：先把 LCD 点亮（顺序与 LAB1 参考例程完全一致）═══
     * 中断必须在 LCD 初始化之前打开：即使 LCD 卡住，串口/按键/时基
     * 仍然正常，不会出现"屏幕黑 + 按键全废 + 串口无响应"的全盘僵死。 */
    __enable_interrupt();

    Dogs102x6_init();
#if BOOT_DIAG_FULL
    /* 背光硬件自检：纯 GPIO 直接拉 P7.6 闪 3 下。
     *   看得到闪动 -> 背光链路（P7.6/TPS75105/升压/排线）是好的，
     *                 屏还黑就只能是数据或对比度问题；
     *   完全看不到 -> 背光链路断了，纯硬件，代码再怎么改都没用。 */
    Dogs102x6_backlightBlink(3);
#endif

    /* 背光：直接用 GPIO 把 P7.6 拉高，常亮 100%，**不再走 TimerB0 的 PWM**。
     *
     * PWM 版本（backlightInit + setBacklight）依赖 TBSSEL_1 = ACLK：
     * ACLK 一旦没起振（XT1/REFOCLK 配置被覆盖、UCSCTL 被意外改写），
     * TB0 不计数，OUTMOD_7 的输出就停在低电平，TPS75105 的 EN 恒为 0，
     * 屏幕立刻变成一块黑 —— 而程序、串口、按键全部正常，极难定位。
     * 电机启停时的电源波动还可能让 ACLK 相关寄存器受扰，触发同样的现象。
     *
     * GPIO 版本没有这条时钟依赖，上电必然点亮；代价是亮度固定 100%
     * （原 PWM 约 90%，肉眼基本看不出来）。
     * 想回到可调亮度，把下面一行换回：
     *     Dogs102x6_backlightInit(); Dogs102x6_setBacklight(11);
     */
    /* 默认关闭背光：DOGS102x6 为反射式 FSTN 正显液晶屏，在室内环境光下自然反射显示效果最佳。
     * 关闭背光可获得纯正银灰底色与深黑文字的极致对比度，彻底消除背光常亮导致的整体泛白模糊。 */
    Dogs102x6_backlightOff();
    Dogs102x6_setContrast(11);      /* 11/31：单片机实验课 LAB1 经典标定高对比度值 */
    Dogs102x6_clearScreen();

#if BOOT_DIAG_FULL
    /* 上电自检图案：满屏棋盘格。只要这屏出现，
     * 就证明 SPI/LCD 数据通路与背光全部正常，剩下的都是数据问题。 */
    for (uint8_t y = 0; y < DOGS102x6_Y_SIZE; y += 8) {
        for (uint8_t x = 0; x < DOGS102x6_X_SIZE; x++) {
            if (((x >> 2) + (y >> 3)) & 1) {
                dogs102x6Memory[2 + (y >> 3) * 102 + x] = 0x00;
            } else {
                dogs102x6Memory[2 + (y >> 3) * 102 + x] = 0xFF;
            }
        }
        Dogs102x6_setAddress(y >> 3, 0);
        Dogs102x6_writeData((uint8_t *)&dogs102x6Memory[2 + (y >> 3) * 102], 102);
    }
    __delay_cycles(15000000);       /* 600ms：让自检图案肉眼可见 */
#endif

#if BOOT_SPLASH_MS > 0
    /* 启动页：确认 LCD 数据通路正常，250ms 一闪而过，不挡操作。 */
    Dogs102x6_clearScreen();
    Dogs102x6_stringDrawBig(3, 26, "LCD OK", DOGS102x6_DRAW_NORMAL);
    __delay_cycles((unsigned long)BOOT_SPLASH_MS * 25000UL);   /* 25MHz: 25000 cycles/ms */
#endif
    Boot_Mark(2);               /* 绿闪 2 次 = LCD 初始化 + 背光完成 */

    Uart1_Init();
    Boot_Mark(3);               /* 绿闪 3 次 = UCA1 串口就绪 */

    Wheel_Init();
    Peripherals_Init();
    Boot_Mark(4);               /* 绿闪 4 次 = 按键/ADC/定时器就绪 */

    Dogs102x6_clearScreen();
    Dogs102x6_horizontalLineDraw(2, 99, 19, DOGS102x6_DRAW_NORMAL);
    Dogs102x6_horizontalLineDraw(2, 99, 43, DOGS102x6_DRAW_NORMAL);
    Update_Dashboard();

    uint32_t last_display = 0U;
    uint32_t last_wheel   = 0U;
    uint32_t last_hb      = 0U;
    uint32_t last_lcd_ok  = 0U;

    while (1) {
        /* 发送泵放在最前面：任何一个 80ms/50ms 的定时任务都不该挡住上行字节 */
        Uart_Poll_Tx();

        /* 接收链路看门狗：清溢出 + 长时间无帧就软复位 UCA1 */
        Uart1_HealthCheck();

        /* LCD 状态保活：每 2s 重发"正常显示 / 电源开 / 开显示"。
         * UC1701x 与电机共用 3.3V，电机堵转/换向造成的电压跌落可能
         * 只把 LCD 控制器复位掉（MCU 没事），复位后默认"显示关"，
         * 屏上就是突然冒出一块黑，但程序照跑。
         * 这几条命令不清显存、不闪屏，控制器一旦被复位会在 2s 内自己恢复，
         * 画面由下一帧 80ms 的整体重绘补齐。 */
        if ((s_system_ticks - last_lcd_ok) >= 2000U) {
            last_lcd_ok = s_system_ticks;
            Dogs102x6_refreshPowerState();
        }

        if (Link_IsUp()) {
            P8OUT |= BIT1;
            P1OUT &= ~BIT0;
        } else {
            P1OUT |= BIT0;
            P8OUT &= ~BIT1;
        }

        /* 心跳：让天猛星知道显示终端在线（不参与上位机连接判定） */
        if ((s_system_ticks - last_hb) >= HB_PERIOD_MS) {
            last_hb = s_system_ticks;
            Uart_PutStr("!HB#");
        }

        /* 按键：消抖已经在 1ms 中断里完成，这里只剩"取事件 -> 发字节"，
         * 不阻塞、不延时，也不会因为抖动一次按键发两个 '1'。
         * 读-清之间必须关中断：否则刚读到的位可能被 ISR 重新置起而丢掉。 */
        if (s_key_event != 0U) {
            uint16_t sr = __get_SR_register() & GIE;
            uint8_t  ev;

            __disable_interrupt();
            ev = s_key_event;
            s_key_event = 0U;
            __bis_SR_register(sr);

            if (ev & 0x01U) {
                Uart_Put((uint8_t)'1');     /* S1：启停切换 */
            }
            if (ev & 0x02U) {
                Uart_Put((uint8_t)'2');     /* S2：里程/航向清零 */
            }
        }

        if ((s_system_ticks - last_wheel) >= 50U) {
            last_wheel = s_system_ticks;
            Wheel_CheckUpdate();
        }

        if ((s_system_ticks - last_display) >= 80U) {
            last_display = s_system_ticks;
            Update_Dashboard();
        }
    }
}
