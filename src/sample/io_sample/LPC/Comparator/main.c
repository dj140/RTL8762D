/**
*********************************************************************************************************
*               Copyright(c) 2018, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     main.c
* @brief    This file provides demo code to lpc comparator.
            When the voltage on the pin is higher than the set value, the counter adds one.
            LPC comparator interruption will occur when the current value of the counter equals the set value.
            LPC comparator interrupt are located in RTC_Handler.
* @details
* @author   yuan
* @date     2018-06-27
* @version  v0.1
*********************************************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include "rtl876x_lpc.h"
#include "rtl876x_nvic.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"
#include "rtl876x_rtc.h"
#include "trace.h"

/* Defines ------------------------------------------------------------------*/
/** LPC voltage detect channel config.
  * PIN select:P2_0~P2_7 and VBAT.
  * LPC channel select: #define LPC_CHANNEL_P2_0                ((uint32_t)0)
  *                     #define LPC_CHANNEL_P2_1                ((uint32_t)1)
  *                     #define LPC_CHANNEL_P2_2                ((uint32_t)2)
  *                     #define LPC_CHANNEL_P2_3                ((uint32_t)3)
  *                     #define LPC_CHANNEL_P2_4                ((uint32_t)4)
  *                     #define LPC_CHANNEL_P2_5                ((uint32_t)5)
  *                     #define LPC_CHANNEL_P2_6                ((uint32_t)6)
  *                     #define LPC_CHANNEL_P2_7                ((uint32_t)7)
  *                     #define LPC_CHANNEL_VBAT                ((uint32_t)8)
  * If LPC_CAPTURE_PIN = P2_0, then LPC_CAPTURE_CHANNEL = LPC_CHANNEL_P2_0;
  * If LPC_CAPTURE_PIN = P2_5, then LPC_CAPTURE_CHANNEL = LPC_CHANNEL_P2_5;
  */
#define LPC_CAPTURE_PIN                 P2_4
#define LPC_CAPTURE_CHANNEL             LPC_CAPTURE_PIN - P2_0

#define LPC_VOLTAGE_DETECT_EDGE         LPC_Vin_Over_Vth;
#define LPC_VOLTAGE_DETECT_THRESHOLD    LPC_2400_mV;

#define LPC_COMP_VALUE                  0x0A    /* A LPC comparator interrupt occurs every ten times */

/**
  * @brief  initialization of pinmux settings and pad settings.
  * @param   No parameter.
  * @return  void
  */
void board_lpc_init(void)
{
    Pad_Config(LPC_CAPTURE_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
               PAD_OUT_HIGH);
    Pinmux_Config(LPC_CAPTURE_PIN, IDLE_MODE);
}

/**
  * @brief  Initialize LPC peripheral.
  * @param   No parameter.
  * @return  void
  */
void driver_lpc_init(void)
{
    LPC_InitTypeDef LPC_InitStruct;

    LPC_StructInit(&LPC_InitStruct);
    LPC_InitStruct.LPC_Channel   = LPC_CAPTURE_CHANNEL;
    LPC_InitStruct.LPC_Edge      = LPC_VOLTAGE_DETECT_EDGE;
    LPC_InitStruct.LPC_Threshold = LPC_VOLTAGE_DETECT_THRESHOLD;
    LPC_Init(&LPC_InitStruct);
    LPC_Cmd(ENABLE);

    LPC_ResetCounter();
    LPC_SetCompValue(LPC_COMP_VALUE);
    LPC_CounterCmd(ENABLE);
    LPC_INTConfig(LPC_INT_LPCOMP_CNT, ENABLE);

    /* Config LPC interrupt */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    RTC_NvCmd(ENABLE);
}

/**
  * @brief  Demo code of operation about lpc.
  * @param  No parameter.
  * @return void
*/
void lpc_demo(void)
{
    /* Configure PAD and pinmux firstly! */
    board_lpc_init();

    /* Initialize LPC peripheral */
    driver_lpc_init();
}

/**
 * @brief    Entry of APP code
 * @return   int (To avoid compile warning)
 */
int main(void)
{
    extern uint32_t random_seed_value;
    srand(random_seed_value);
    __enable_irq();

    lpc_demo();

    while (1)
    {
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();

    }
}

/**
  * @brief  RTC interrupt handle function.
  * @param  None.
  * @return None.
  */
void RTC_Handler(void)
{
    /* LPC counter comparator interrupt */
    if (LPC_GetINTStatus(LPC_INT_LPCOMP_CNT) == SET)
    {
        /* Notes: DBG_DIRECT is only used in debug demo, do not use in app project.*/
        DBG_DIRECT("LPC_INT_LPCOMP_CNT");
        LPC_SetCompValue(LPC_GetCounter() + LPC_COMP_VALUE);
        LPC_ClearINTPendingBit(LPC_INT_LPCOMP_CNT);
    }
}

/******************* (C) COPYRIGHT 2020 Realtek Semiconductor Corporation *****END OF FILE****/

