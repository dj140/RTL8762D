/**
*********************************************************************************************************
*               Copyright(c) 2018, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     io_uart.c
* @brief    uart interrupt demo
* @details
* @author   yuan
* @date     2018-06-28
* @version  v0.1
*********************************************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "io_adc.h"
#include <string.h>
#include "app_task.h"

/* Globals ------------------------------------------------------------------*/
uint16_t ADC_DATA_Buffer[32];
uint8_t ADC_DATA_Length = 0;

bool IO_ADC_DLPS_Enter_Allowed = false;

/**
  * @brief  Initialize uart global data.
  * @param  No parameter.
  * @return void
  */
void global_data_adc_init(void)
{
    IO_ADC_DLPS_Enter_Allowed = true;
    ADC_DATA_Length = 0;
    memset(ADC_DATA_Buffer, 0, sizeof(ADC_DATA_Buffer));
}

void board_adc_init(void)
{
    Pad_Config(ADC_SAMPLE_PIN_0, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
               PAD_OUT_LOW);

    Pad_Config(ADC_SAMPLE_PIN_1, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
               PAD_OUT_LOW);
}

/**
  * @brief  Initialize adc peripheral.
  * @param  No parameter.
  * @return void
*/
void driver_adc_init(void)
{
    RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, ENABLE);

    ADC_InitTypeDef ADC_InitStruct;
    ADC_StructInit(&ADC_InitStruct);

    ADC_InitStruct.ADC_SampleTime       = 255;  /* (n + 1) cycle of 10MHz,n = 0~255 or n = 2048~14591 */

    ADC_InitStruct.ADC_SchIndex[0]      = INTERNAL_VBAT_MODE;
    ADC_InitStruct.ADC_Bitmap           = 0x01;
    ADC_InitStruct.ADC_DataAvgEn        = ADC_DATA_AVERAGE_ENABLE;
    ADC_InitStruct.ADC_DataAvgSel       = ADC_DATA_AVERAGE_OF_4;

//    ADC_InitStruct.ADC_TimerTriggerEn   = ADC_TIMER_TRIGGER_ENABLE;
    ADC_InitStruct.ADC_PowerAlwaysOnEn  = ADC_POWER_ALWAYS_ON_ENABLE;
    ADC_Init(ADC, &ADC_InitStruct);

#if (ADC_MODE_DIVIDE_OR_BYPASS == ADC_BYPASS_MODE)
    /* High bypass resistance mode config, please notice that the input voltage of
      adc channel using high bypass mode should not be over 0.9V */
    ADC_BypassCmd(ADC_SAMPLE_CHANNEL_0, ENABLE);
    ADC_BypassCmd(ADC_SAMPLE_CHANNEL_1, ENABLE);
#endif

    ADC_INTConfig(ADC, ADC_INT_ONE_SHOT_DONE, ENABLE);
//    ADC_INTConfig(ADC, ADC_INT_FIFO_RD_ERR, ENABLE);
//    ADC_INTConfig(ADC, ADC_INT_FIFO_OVERFLOW, ENABLE);

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = ADC_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

/**
  * @brief  Initialize timer peripheral for adc one shot mode.
  * @param  No parameter.
  * @return void
*/
void driver_adc_timer_init(void)
{
    /* TIM7 for timing voltage acqusition(fix) */
    RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_InitStruct;

    TIM_StructInit(&TIM_InitStruct);

    TIM_InitStruct.TIM_PWM_En = PWM_DISABLE;
    TIM_InitStruct.TIM_Period = TIMER_ADC_ONE_SHOT_SAMPLE_PERIOD - 1 ;    //sampling once 1 second
    TIM_InitStruct.TIM_Mode = TIM_Mode_UserDefine;
    TIM_InitStruct.TIM_SOURCE_DIV = TIM_CLOCK_DIVIDER_40;
    TIM_TimeBaseInit(TIM7, &TIM_InitStruct);

    TIM_ClearINT(TIM7);
    TIM_Cmd(TIM7, ENABLE);
}

/**
  * @brief  Initialization of pinmux settings and pad settings.
  * @param  No parameter.
  * @return void
  */
void board_gpio_init(void)
{
    Pad_Config(ADC_DLPS_WAKEUP_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
               PAD_OUT_HIGH);

    Pinmux_Config(ADC_DLPS_WAKEUP_PIN, DWGPIO);
}

/**
  * @brief  Initialize GPIO peripheral.
  * @param  No parameter.
  * @return void
  */
void driver_gpio_init(void)
{
    /* Initialize GPIO peripheral */
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin        = GPIO_PIN_INPUT;
    GPIO_InitStruct.GPIO_Mode       = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_ITCmd      = ENABLE;
    GPIO_InitStruct.GPIO_ITTrigger  = GPIO_INT_Trigger_EDGE;
    GPIO_InitStruct.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
    GPIO_InitStruct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
    GPIO_InitStruct.GPIO_DebounceTime = 64;/* unit:ms , can be 1~64 ms */
    GPIO_Init(&GPIO_InitStruct);

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = GPIO_PIN_INPUT_IRQN;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    GPIO_MaskINTConfig(GPIO_PIN_INPUT, DISABLE);
    GPIO_INTConfig(GPIO_PIN_INPUT, ENABLE);
}

/**
  * @brief  IO enter dlps call back function.
  * @param  No parameter.
  * @return void
  */
void io_adc_dlps_enter(void)
{
    uint8_t reg_value = 0;
    reg_value = btaon_fast_read_safe(0x113);
    btaon_fast_write(0x113, reg_value | 0x04);
    /* Switch pad to Software mode */
    Pad_ControlSelectValue(ADC_DLPS_WAKEUP_PIN, PAD_SW_MODE);
    System_WakeUpPinEnable(ADC_DLPS_WAKEUP_PIN, PAD_WAKEUP_POL_LOW, 0);
    DBG_DIRECT("io_adc_dlps_enter");
}

/**
  * @brief  IO exit dlps call back function.
  * @param  No parameter.
  * @return void
  */
void io_adc_dlps_exit(void)
{
    uint8_t reg_value = 0;
    reg_value = btaon_fast_read_safe(0x113);
    btaon_fast_write(0x113, reg_value & (~0x04));
    /* Switch pad to Pinmux mode */
    Pad_ControlSelectValue(ADC_DLPS_WAKEUP_PIN, PAD_PINMUX_MODE);
    DBG_DIRECT("io_adc_dlps_exit");
}

/**
  * @brief  IO enter dlps check function.
  * @param  No parameter.
  * @return void
  */
bool io_adc_dlps_check(void)
{
    return IO_ADC_DLPS_Enter_Allowed;
}

/**
  * @brief  Handle uart data function.
  * @param  No parameter.
  * @return void
  */
void io_adc_handle_msg(T_IO_MSG *io_adc_msg)
{
    uint16_t *p_buf = io_adc_msg->u.buf;
    uint16_t type = io_adc_msg->type;

    if (IO_MSG_TYPE_ADC == type)
    {
        DBG_DIRECT("io_adc_handle_msg: len = %d", ADC_DATA_Length);
        for (uint8_t i = 0; i < ADC_DATA_Length; i++)
        {
            DBG_DIRECT("adc raw data = 0x%X", p_buf[i]);
        }
        platform_delay_ms(1000);
        IO_ADC_DLPS_Enter_Allowed = true;
    }
}

/**
  * @brief  Handle uart msg function.
  * @param  No parameter.
  * @return void
  */
void io_handle_adc_msg(T_IO_MSG *io_adc_msg)
{
    io_adc_handle_msg(io_adc_msg);
}

/**
  * @brief  IO enter dlps check function.
  * @param  No parameter.
  * @return void
  */
void io_adc_sample_start(void)
{
    ADC_Cmd(ADC, ADC_ONE_SHOT_MODE, ENABLE);
}

/**
  * @brief  ADC interrupt handler function.
  * @param  No parameter.
  * @return void
*/
void ADC_Handler(void)
{
    uint16_t sample_data[32];

    if (ADC_GetINTStatus(ADC, ADC_INT_ONE_SHOT_DONE) == SET)
    {
        ADC_ClearINTPendingBit(ADC, ADC_INT_ONE_SHOT_DONE);

        /* Send msg to app task */
        ADC_DATA_Length = 1;
        sample_data[0] = ADC_ReadRawData(ADC, 0);
        ADC_DATA_Buffer[0] = sample_data[0];
        T_IO_MSG int_adc_msg;

        int_adc_msg.type = IO_MSG_TYPE_ADC;
        int_adc_msg.u.buf = (void *)(&ADC_DATA_Buffer);
        if (false == app_send_msg_to_apptask(&int_adc_msg))
        {
            APP_PRINT_ERROR0("[io_adc] ADC_Handler: Send int_adc_msg failed!");
            //Add user code here!
            return;
        }
    }
}

/**
  * @brief  GPIO interrupt handler function.
  * @param  No parameter.
  * @return void
  */
void GPIO_Input_Handler(void)
{
    DBG_DIRECT("GPIO_Input_Handler");
    GPIO_INTConfig(GPIO_PIN_INPUT, DISABLE);
    GPIO_MaskINTConfig(GPIO_PIN_INPUT, ENABLE);

    T_IO_MSG int_gpio_msg;

    int_gpio_msg.type = IO_MSG_TYPE_GPIO;
    int_gpio_msg.subtype = 0;
    if (false == app_send_msg_to_apptask(&int_gpio_msg))
    {
        APP_PRINT_ERROR0("[io_gpio] GPIO_Input_Handler: Send int_gpio_msg failed!");
        //Add user code here!
        GPIO_ClearINTPendingBit(GPIO_PIN_INPUT);
        return;
    }

    GPIO_ClearINTPendingBit(GPIO_PIN_INPUT);
    GPIO_MaskINTConfig(GPIO_PIN_INPUT, DISABLE);
    GPIO_INTConfig(GPIO_PIN_INPUT, ENABLE);
}

/******************* (C) COPYRIGHT 2018 Realtek Semiconductor Corporation *****END OF FILE****/
