/**
*********************************************************************************************************
*               Copyright(c) 2018, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file     io_adc.c
* @brief    This file provides demo code of adc continuous mode.
* @details
* @author   yuan
* @date     2018-12-07
* @version  v1.0
*********************************************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "io_adc.h"

#include "app_task.h"

/* Globals ------------------------------------------------------------------*/
ADC_Data_TypeDef ADC_Global_Data;

/**
  * @brief  Initialization adc global data.
  * @param  No parameter.
  * @return void
  */
void global_data_adc_init(void)
{
    /* Initialize adc k value! */
    APP_PRINT_INFO0("[io_adc] global_data_adc_init");
    bool adc_k_status = false;
    adc_k_status = ADC_CalibrationInit();
    if (false == adc_k_status)
    {
        APP_PRINT_ERROR0("[io_adc] global_data_adc_init: ADC_CalibrationInit fail!");
    }
    memset(&ADC_Global_Data, 0, sizeof(ADC_Global_Data));
}

/**
  * @brief  Initialization of pinmux settings and pad settings.
  * @param  No parameter.
  * @return void
*/
void board_adc_init(void)
{
    Pad_Config(ADC_SAMPLE_PIN_0, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
               PAD_OUT_LOW);
}

/**
  * @brief  Initialize ADC peripheral.
  * @param  No parameter.
  * @return void
  */
void driver_adc_init(void)
{
    RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, ENABLE);

    ADC_InitTypeDef ADC_InitStruct;
    ADC_StructInit(&ADC_InitStruct);

    /* Configure the ADC sampling schedule0 */
    ADC_InitStruct.ADC_SchIndex[0]      = EXT_SINGLE_ENDED(ADC_SAMPLE_CHANNEL_0);
    /* Set the bitmap corresponding to schedule0*/
    ADC_InitStruct.ADC_Bitmap           = 0x01;

#if (ADC_DATA_HW_AVERAGE && ADC_DATA_OUTPUT_TO_FIFO)
    APP_PRINT_ERROR0("[io_adc] driver_adc_init: ADC config error !");
#elif (ADC_DATA_HW_AVERAGE )
    ADC_InitStruct.ADC_DataAvgEn        = ADC_DATA_AVERAGE_ENABLE;
    ADC_InitStruct.ADC_DataAvgSel       = ADC_DATA_AVERAGE_OF_4;
#elif (ADC_DATA_OUTPUT_TO_FIFO)
    ADC_InitStruct.ADC_DataWriteToFifo  = ADC_DATA_WRITE_TO_FIFO_ENABLE;
    ADC_InitStruct.ADC_FifoThdLevel     = 0x0A;
#endif

    ADC_InitStruct.ADC_PowerAlwaysOnEn  = ADC_POWER_ALWAYS_ON_ENABLE;

#if (ADC_DATA_HW_AVERAGE || ADC_DATA_OUTPUT_TO_FIFO || ADC_MULTI_SAMPLE)
    /* In one shot mode, if it is necessary to sample at regular intervals,
       the hardware timer TIM7 (which needs to be configured separately) can be used to realize ADC timing sampling.
       After enabling the timer, there is no need to manually enable ADC for each sampling.*/
    ADC_InitStruct.ADC_TimerTriggerEn   = ADC_TIMER_TRIGGER_ENABLE;
#endif
    /* Fixed 255 in OneShot mode. */
    ADC_InitStruct.ADC_SampleTime       = 255;

    ADC_Init(ADC, &ADC_InitStruct);

#if (ADC_MODE_DIVIDE_OR_BYPASS == ADC_BYPASS_MODE)
    /* High bypass resistance mode config, please notice that the input voltage of
      adc channel using high bypass mode should not be over 0.9V */
    ADC_BypassCmd(ADC_SAMPLE_CHANNEL_0, ENABLE);
    APP_PRINT_INFO0("[io_adc] driver_adc_init: ADC sample mode is bypass mode !");
#else
    ADC_BypassCmd(ADC_SAMPLE_CHANNEL_0, DISABLE);
    APP_PRINT_INFO0("[io_adc] driver_adc_init: ADC sample mode is divide mode !");
#endif

#if (!ADC_DATA_OUTPUT_TO_FIFO)
    ADC_INTConfig(ADC, ADC_INT_ONE_SHOT_DONE, ENABLE);
#else
    ADC_INTConfig(ADC, ADC_INT_FIFO_THD, ENABLE);
#endif

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = ADC_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    /* When ADC is enabled, sampling will be done quickly and interruption will occur.
       After initialization, ADC can be enabled when sampling is needed.*/
//    ADC_Cmd(ADC, ADC_ONE_SHOT_MODE, ENABLE);
}

/**
  * @brief  Initialize timer peripheral for adc one shot mode.
  * @param  No parameter.
  * @return void
*/
void driver_adc_timer_init(void)
{
    /* Only timer7 can be used as a hardware timer for timing sampling of ADC one shot mode. */
    RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_InitStruct;

    TIM_StructInit(&TIM_InitStruct);

    TIM_InitStruct.TIM_PWM_En = PWM_DISABLE;
    /* Timing settings for timer see IO TIM demo. */
    TIM_InitStruct.TIM_Period = ADC_ONE_SHOT_SAMPLE_PERIOD - 1 ;    //sampling once 1 second
    TIM_InitStruct.TIM_Mode = TIM_Mode_UserDefine;
    TIM_InitStruct.TIM_SOURCE_DIV = TIM_CLOCK_DIVIDER_40;
    TIM_TimeBaseInit(TIM7, &TIM_InitStruct);

    TIM_ClearINT(TIM7);
}

/**
  * @brief  Calculate adc sample voltage.
  * @param  No parameter.
  * @return void
  */
static void io_adc_voltage_calculate(T_IO_MSG *io_adc_msg)
{
    ADC_Data_TypeDef *p_buf = io_adc_msg->u.buf;
    uint8_t sample_data_len = 0;
    uint16_t sample_data = 0;

    sample_data_len = p_buf->RawDataLen;
    for (uint8_t i = 0; i < sample_data_len; i++)
    {
        sample_data = p_buf->RawData[i];
        DBG_DIRECT("io_adc_voltage_calculate: raw_data = 0x%X", sample_data);
#if (ADC_DATA_HW_AVERAGE )
        sample_data = (p_buf->RawData[i] & 0x3FFC) >> 2;
        uint16_t sample_data_decimal = (p_buf->RawData[i] & 0x3);

        float cacl_result = sample_data;
        float cacl_result_dec = 0;
        cacl_result_dec = (float)(sample_data_decimal & 0x1) / 2 + (float)((sample_data_decimal >> 1) & 0x1)
                          / 2;
        cacl_result += cacl_result_dec;
        DBG_DIRECT("io_adc_voltage_calculate: sample_data = %d, cacl_result = %f\r\n",
                   sample_data, cacl_result);

#endif
//        float sample_voltage = 0;
//        ADC_ErrorStatus error_status = NO_ERROR;
//#if (ADC_MODE_DIVIDE_OR_BYPASS == ADC_BYPASS_MODE)
//        sample_voltage = ADC_GetVoltage(BYPASS_SINGLE_MODE, (int32_t)sample_data, &error_status);
//#else
//        sample_voltage = ADC_GetVoltage(DIVIDE_SINGLE_MODE, (int32_t)sample_data, &error_status);
//#endif
//        if (error_status < 0)
//        {
//            APP_PRINT_INFO1("[io_adc] io_adc_voltage_calculate: ADC parameter or efuse data error! error_status = %d",
//                            error_status);
//        }
//        else
//        {
//            APP_PRINT_INFO2("[io_adc] io_adc_voltage_calculate: ADC rawdata = %d, voltage = %dmV ",
//                            sample_data, (uint32_t)sample_voltage);
//        }
    }
    memset(&ADC_Global_Data, 0, sizeof(ADC_Global_Data));
}

void adc_demo_driver_init(void)
{
    driver_adc_init();
#if (ADC_DATA_HW_AVERAGE || ADC_DATA_OUTPUT_TO_FIFO || ADC_MULTI_SAMPLE)
    driver_adc_timer_init();
#endif
}

void adc_sample_start(void)
{
#if (ADC_DATA_OUTPUT_TO_FIFO)
    ADC_ClearFifo(ADC);
#endif
    /* Enable adc sample */
    ADC_Cmd(ADC, ADC_ONE_SHOT_MODE, ENABLE);

#if (ADC_DATA_HW_AVERAGE || ADC_DATA_OUTPUT_TO_FIFO || ADC_MULTI_SAMPLE)
    TIM_Cmd(TIM7, ENABLE);
#endif
}

/**
  * @brief  Handle adc data function.
  * @param  No parameter.
  * @return void
  */
void io_handle_adc_msg(T_IO_MSG *io_adc_msg)
{
    io_adc_voltage_calculate(io_adc_msg);
}

/**
  * @brief  ADC interrupt handler function.
  * @param  No parameter.
  * @return void
  */
void ADC_Handler(void)
{
#if (!ADC_DATA_OUTPUT_TO_FIFO)
    if (ADC_GetINTStatus(ADC, ADC_INT_ONE_SHOT_DONE) == SET)
    {
        ADC_ClearINTPendingBit(ADC, ADC_INT_ONE_SHOT_DONE);

        uint16_t sample_data = 0;
        sample_data = ADC_ReadRawData(ADC, ADC_Schedule_Index_0);

        T_IO_MSG int_adc_msg;
        int_adc_msg.type = IO_MSG_TYPE_ADC;
        int_adc_msg.subtype = 0;
        ADC_Global_Data.RawDataLen = 1;
        ADC_Global_Data.RawData[0] = sample_data;
        int_adc_msg.u.buf = (void *)(&ADC_Global_Data);
        if (false == app_send_msg_to_apptask(&int_adc_msg))
        {
            APP_PRINT_ERROR0("[io_adc] ADC_Handler: Send int_adc_msg failed!");
            //Add user code here!
            TIM_Cmd(TIM7, DISABLE);
            ADC_ClearINTPendingBit(ADC, ADC_INT_ONE_SHOT_DONE);
            return;
        }
    }
#else
    if (ADC_GetIntFlagStatus(ADC, ADC_INT_FIFO_THD) == SET)
    {
//        uint16_t sample_data[32] = {0};
        TIM_Cmd(TIM7, DISABLE);
//        /* stop write fifo */
//        ADC_ClearINTPendingBit(ADC, ADC_INT_FIFO_THD);
        ADC_Global_Data.RawDataLen = ADC_GetFifoLen(ADC);
        ADC_GetFifoData(ADC, ADC_Global_Data.RawData, ADC_Global_Data.RawDataLen);

        DBG_DIRECT("ADC_INT_FIFO_THD: len = %d", ADC_Global_Data.RawDataLen);
        for (uint8_t i = 0; i < ADC_Global_Data.RawDataLen; i++)
        {
            DBG_DIRECT("ADC_INT_FIFO_THD: data[%d] = %d", i, ADC_Global_Data.RawData[i]);
        }
        /* Send msg to app task */
        T_IO_MSG int_adc_msg;
        int_adc_msg.type = IO_MSG_TYPE_ADC;
        int_adc_msg.u.buf = (void *)(&ADC_Global_Data);
        if (false == app_send_msg_to_apptask(&int_adc_msg))
        {
            APP_PRINT_ERROR0("[io_adc] ADC_Handler: Send int_adc_msg failed!");
            //Add user code here!
            TIM_Cmd(TIM7, DISABLE);
            ADC_ClearFifo(ADC);
            ADC_ClearINTPendingBit(ADC, ADC_INT_FIFO_THD);
            return;
        }
        ADC_ClearINTPendingBit(ADC, ADC_INT_FIFO_THD);
        ADC_ClearFifo(ADC);
        TIM_Cmd(TIM7, ENABLE);
    }
#endif
}

