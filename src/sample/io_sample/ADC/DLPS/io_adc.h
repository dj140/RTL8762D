/**
*********************************************************************************************************
*               Copyright(c) 2018, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file     io_adc.h
* @brief
* @details
* @author   yuan
* @date     2018-12-07
* @version  v1.0
*********************************************************************************************************
*/

#ifndef __IO_ADC_H
#define __IO_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "rtl876x_adc.h"
#include "rtl876x_gpio.h"
#include "rtl876x_io_dlps.h"
#include "rtl876x_nvic.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"
#include "rtl876x_tim.h"

#include "app_msg.h"
#include "board.h"

/* Defines --------------------------------------------------------------*/
#define ADC_SAMPLE_CHANNEL_0                ADC_SAMPLE_PIN_0 - P2_0
#define ADC_SAMPLE_CHANNEL_1                ADC_SAMPLE_PIN_1 - P2_0

/* Config ADC channel and mode --------------------------------------------------------------*/
/* ADC bypass mode or divide mode */
#define ADC_DIVIDE_MODE                     0
#define ADC_BYPASS_MODE                     1
#define ADC_MODE_DIVIDE_OR_BYPASS           ADC_DIVIDE_MODE

#define TIMER_ADC_ONE_SHOT_SAMPLE_PERIOD    (100)//100us

/* Globals ------------------------------------------------------------------*/
extern bool IO_ADC_DLPS_Enter_Allowed;

void global_data_adc_init(void);
void board_adc_init(void);
void driver_adc_init(void);
void board_gpio_init(void);
void driver_gpio_init(void);
void io_adc_dlps_enter(void);
void io_adc_dlps_exit(void);
bool io_adc_dlps_check(void);
void io_handle_adc_msg(T_IO_MSG *io_adc_msg);
void io_adc_sample_start(void);



#ifdef __cplusplus
}
#endif

#endif

