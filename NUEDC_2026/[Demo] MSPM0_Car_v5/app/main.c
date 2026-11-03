#include "app_device_check.h"
#include "app_launcher.h"
#include "app_remote_car_task.h"
#include "bsp_time.h"
#include "canmv_uart.h"
#include "chassis.h"
#include "gimbal.h"
#include "key.h"
#include "line_follow.h"
#include "system_fault.h"
#include "ti_msp_dl_config.h"
#include "ui.h"

#include <stdint.h>

/* ═══════════════════ 通信波特率：单一事实来源 ═══════════════════
 * 天猛星 UART0 / UART2 ↔ MSP430 ↔ 无线透传模块 三方必须完全一致。
 * 这里在 SYSCFG_DL_init() 之后强制重写分频寄存器，因此即使有人重新
 * Generate SysConfig（.syscfg 里曾出现的 9600/115200 冲突），
 * 也不会在不知不觉中把链路波特率改掉。
 *
 *   32MHz BUSCLK 分频值对照：
 *     115200 -> IBRD 17 , FBRD 23
 *      57600 -> IBRD 34 , FBRD 46
 *       9600 -> IBRD 208, FBRD 21
 * 换模块时只改这三行，并同步修改 MSP430 main.c 的 UCAxBR0/UCAxMCTL。 */
#define COMM_UART_BAUD   115200U
#define COMM_UART_IBRD   17U
#define COMM_UART_FBRD   23U

volatile uint32_t g_app_debug_uart_irq_count;
volatile uint32_t g_app_debug_uart_rx_irq_count;
volatile uint32_t g_app_debug_uart_drained_byte_count;
volatile uint32_t g_app_debug_uart_empty_rx_irq_count;
volatile uint32_t g_app_debug_uart_unhandled_irq_count;
volatile uint8_t g_app_debug_uart_last_iidx;

/* ═══════════════════ 三路串口职责（按实车接线）═══════════════════════════
 *   UART1  PA8(TX) / PA9(RX)   -> 无线透传模块 -> PC 上位机       [Wireless_INST]
 *   UART0  PB0(TX) / PB1(RX)   -> MSP430F5529 显示终端 UCA1        [Debug_INST]
 *   UART2  PA24(RX)/PB15(TX)   -> 视觉 K230（本车未接，保留不发送）
 * 旧工程把 UART0 误当"蓝牙口"，而无线模块真正所在的 PA8/PA9 完全没有驱动，
 * 这就是"两个无线模块互测能通、一装到车上就收不到命令"的直接原因。
 * ═══════════════════════════════════════════════════════════════════════ */
static void App_DrainDebugUartRxFifo(void){
    uint32_t drained = 0U;

    while (!DL_UART_Main_isRXFIFOEmpty(Debug_INST)){
        uint8_t byte = (uint8_t)DL_UART_Main_receiveData(Debug_INST);
        AppRemoteCar_ProcessMsp430Byte(byte);
        drained++;
    }

    g_app_debug_uart_drained_byte_count += drained;
    if (drained == 0U){
        g_app_debug_uart_empty_rx_irq_count++;
    }
}

static void App_DrainWirelessUartRxFifo(void){
    while (!DL_UART_Main_isRXFIFOEmpty(Wireless_INST)){
        uint8_t byte = (uint8_t)DL_UART_Main_receiveData(Wireless_INST);
        AppRemoteCar_ProcessPcByte(byte);
    }
}

static void App_InitSystems(void){
    SYSCFG_DL_init();

    /* 强制锁定通信波特率，避免 SysConfig 重新生成时被改回 9600 */
    DL_UART_Main_setBaudRateDivisor(Debug_INST, COMM_UART_IBRD, COMM_UART_FBRD);
    DL_UART_Main_setBaudRateDivisor(Wireless_INST, COMM_UART_IBRD, COMM_UART_FBRD);
    DL_UART_Main_setBaudRateDivisor(CANMV_UART_INST, COMM_UART_IBRD, COMM_UART_FBRD);

    BSP_Time_Init();
    __enable_irq();

    DL_UART_Main_setRXFIFOThreshold(Debug_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setRXInterruptTimeout(Debug_INST, 1U);
    DL_UART_Main_enableInterrupt(Debug_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    NVIC_SetPriority(Debug_INST_INT_IRQN, 1);
    NVIC_ClearPendingIRQ(Debug_INST_INT_IRQN);
    NVIC_EnableIRQ(Debug_INST_INT_IRQN);

    /* 无线模块(UART1)：上位机遥控指令的唯一入口 */
    DL_UART_Main_setRXFIFOThreshold(Wireless_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setRXInterruptTimeout(Wireless_INST, 1U);
    DL_UART_Main_enableInterrupt(Wireless_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    NVIC_SetPriority(Wireless_INST_INT_IRQN, 1);
    NVIC_ClearPendingIRQ(Wireless_INST_INT_IRQN);
    NVIC_EnableIRQ(Wireless_INST_INT_IRQN);

    Ui_Init();
    Key_Init();

    (void)Chassis_Init();
    (void)Gimbal_Init();
    (void)LineFollow_Init();
    (void)CanMvUart_Init();
    SystemFault_Clear();

    DL_TimerA_startCounter(TIMER_0_INST);
}

int main(void){
    App_InitSystems();
    App_Launch();

    while (1){
    }
}

void UART0_IRQHandler(void){
    DL_UART_IIDX iidx = DL_UART_getPendingInterrupt(Debug_INST);

    g_app_debug_uart_irq_count++;
    g_app_debug_uart_last_iidx = (uint8_t)iidx;

    /* 无论何种中断，无条件排空 RX FIFO 中的字节，防止未决字节导致重入死循环 */
    App_DrainDebugUartRxFifo();

    /* 立即清除任何可能触发的硬件错误标志（Overrun/Noise/Framing/Parity），杜绝中断风暴 */
    DL_UART_clearInterruptStatus(Debug_INST,
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
}

void UART1_IRQHandler(void){
    (void)DL_UART_getPendingInterrupt(Wireless_INST);

    /* 同样无条件排空 RX FIFO，防止未决字节导致中断反复重入 */
    App_DrainWirelessUartRxFifo();

    DL_UART_clearInterruptStatus(Wireless_INST,
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
}

void UART2_IRQHandler(void){
    DL_UART_IIDX iidx = DL_UART_getPendingInterrupt(CANMV_UART_INST);

    /* 无论何种中断，无条件排空 RX FIFO 中的字节，防止未决字节导致重入死循环 */
    CanMvUart_ProcessRx();

    /* 立即清除任何可能触发的硬件错误标志（Overrun/Noise/Framing/Parity/Break），杜绝中断风暴 */
    DL_UART_clearInterruptStatus(CANMV_UART_INST,
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
}

void SysTick_Handler(void){
    static uint8_t scan_divider = 0U;

    BSP_Time_TickInc();

    scan_divider++;
    if (scan_divider >= 10U){
        Key_Scan();
        scan_divider = 0U;
    }
}
