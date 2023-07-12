/**
*********************************************************************************************************
*               Copyright(c) 2021, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file     io_i8080.h
* @brief
* @details
* @author   yuan
* @date     2021-01-08
* @version  v1.0.0
*********************************************************************************************************
*/

#ifndef __IO_I8080_H
#define __IO_I8080_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include "rtl876x_gdma.h"
#include "rtl876x_gpio.h"
#include "rtl876x_if8080.h"
#include "rtl876x_nvic.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"


/* Defines ------------------------------------------------------------------*/
/* Select the  mode: vsync mode or auto mode */
#define I8080_GDMA_AUTO_MODE_DEMO    0
#if I8080_GDMA_AUTO_MODE_DEMO
#define I8080_GDMA_DEMO         1
#else
#define I8080_VSYNC_DEMO        1
#endif

#define IF8080_PINGROUP_1       0
#if IF8080_PINGROUP_1
#define LCD_8080_RST                     P2_0
#define LCD_8080_BL                      P3_6 //P1_5 CK NO THIS PIN
/*because most 8080 lcd pins ara fixed*/
#define LCD_8080_D0                      P3_5
#define LCD_8080_D1                      P0_1
#define LCD_8080_D2                      P0_2
#define LCD_8080_D3                      P0_4
#define LCD_8080_D4                      P4_0
#define LCD_8080_D5                      P4_1
#define LCD_8080_D6                      P4_2
#define LCD_8080_D7                      P4_3

#define LCD_8080_CS                      P3_3
#define LCD_8080_DCX                     P3_4
#define LCD_8080_RD                      P2_0
#define LCD_8080_WR                      P3_2
#define LCD_FRAME_SYNC                   P1_2

#else

#define LCD_8080_RST                     P2_0
#define LCD_8080_BL                      P1_2 //need R15
/*because most 8080 lcd pins ara fixed*/
#define LCD_8080_D0                      P0_4
#define LCD_8080_D1                      P0_5
#define LCD_8080_D2                      P0_6
#define LCD_8080_D3                      P0_7
#define LCD_8080_D4                      P4_0
#define LCD_8080_D5                      P4_1
#define LCD_8080_D6                      P4_2
#define LCD_8080_D7                      P4_3

#define LCD_8080_CS                      P0_0
#define LCD_8080_DCX                     P1_5
#define LCD_8080_RD                      P1_6
#define LCD_8080_WR                      P0_2
#define LCD_FRAME_SYNC                   P0_1

#define GPIO_FRAME_SYNC_IRQ              GPIO1_IRQn
#define BL_PWM_TIM                       TIM4
#endif

void i8080_demo(void);


#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2021 Realtek Semiconductor Corporation *****END OF FILE****/
