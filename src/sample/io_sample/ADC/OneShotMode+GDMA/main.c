/**
*********************************************************************************************************
*               Copyright(c) 2018, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     main.c
* @brief
* @details
* @author   yuan
* @date     2018-06-22
* @version  v0.1
*********************************************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "rtl876x_adc.h"
#include "rtl876x_gdma.h"
#include "rtl876x_nvic.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"
#include "rtl876x_tim.h"
#include "trace.h"

/* Defines --------------------------------------------------------------*/
#define ADC_Channel_Index_0         0
#define ADC_Channel_Index_1         1
#define ADC_Channel_Index_2         2
#define ADC_Channel_Index_3         3
#define ADC_Channel_Index_4         4
#define ADC_Channel_Index_5         5
#define ADC_Channel_Index_6         6
#define ADC_Channel_Index_7         7

#define ADC_Schedule_Index_0        0
#define ADC_Schedule_Index_1        1
#define ADC_Schedule_Index_2        2
#define ADC_Schedule_Index_3        3
#define ADC_Schedule_Index_4        4
#define ADC_Schedule_Index_5        5
#define ADC_Schedule_Index_6        6
#define ADC_Schedule_Index_7        7
#define ADC_Schedule_Index_8        8
#define ADC_Schedule_Index_9        9

/* Config ADC channel and mode --------------------------------------------------------------*/
//ADC bypass mode or divide mode
#define ADC_DIVIDE_MODE                     0
#define ADC_BYPASS_MODE                     1
#define ADC_MODE_DIVIDE_OR_BYPASS           ADC_DIVIDE_MODE

//ADC sample channel config:P2_0~P2_7 VBAT
#define ADC_SAMPLE_PIN_0                    P2_2
#define ADC_SAMPLE_PIN_1                    P2_3
#define ADC_SAMPLE_CHANNEL_0                ADC_Channel_Index_2
#define ADC_SAMPLE_CHANNEL_1                ADC_Channel_Index_3

#define ADC_GDMA_CHANNEL_NUM                3
#define ADC_GDMA_Channel                    GDMA_Channel3
#define ADC_GDMA_Channel_IRQn               GDMA0_Channel3_IRQn
#define ADC_GDMA_Channel_Handler            GDMA0_Channel3_Handler
#define GDMA_TRANSFER_SIZE                  4095

#define TIMER_ADC_ONE_SHOT_SAMPLE_PERIOD    (100)//100us


/* Globals --------------------------------------------------------------*/
uint16_t ADC_Recv_Data_Buffer[GDMA_TRANSFER_SIZE];

/* Functions --------------------------------------------------------------*/
/**
  * @brief  Initialization of pinmux settings and pad settings.
  * @param  No parameter.
  * @return void
*/
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

    ADC_InitStruct.ADC_DataWriteToFifo  = ADC_DATA_WRITE_TO_FIFO_ENABLE;
    ADC_InitStruct.ADC_WaterLevel       = 0x4;
    ADC_InitStruct.ADC_SchIndex[0]      = EXT_SINGLE_ENDED(ADC_SAMPLE_CHANNEL_0);
    ADC_InitStruct.ADC_SchIndex[1]      = EXT_SINGLE_ENDED(ADC_SAMPLE_CHANNEL_1);
    ADC_InitStruct.ADC_Bitmap           = 0x03;
    ADC_InitStruct.ADC_TimerTriggerEn   = ADC_TIMER_TRIGGER_ENABLE;
    ADC_InitStruct.ADC_PowerAlwaysOnEn  = ADC_POWER_ALWAYS_ON_ENABLE;
    ADC_InitStruct.ADC_SampleTime       = 20 -
                                          1; /* (n + 1) cycle of 10MHz,n = 0~255 or n = 2048~14591 */

    ADC_Init(ADC, &ADC_InitStruct);

#if (ADC_MODE_DIVIDE_OR_BYPASS == ADC_BYPASS_MODE)
    /* High bypass resistance mode config, please notice that the input voltage of
      adc channel using high bypass mode should not be over 0.9V */
    ADC_BypassCmd(ADC_SAMPLE_CHANNEL_0, ENABLE);
    ADC_BypassCmd(ADC_SAMPLE_CHANNEL_1, ENABLE);
#endif

    ADC_INTConfig(ADC, ADC_INT_FIFO_RD_ERR, ENABLE);
    ADC_INTConfig(ADC, ADC_INT_FIFO_OVERFLOW, ENABLE);

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
  * @brief  Initialize gdma peripheral.
  * @param  No parameter.
  * @return void
*/
void driver_adc_gdma_init(void)
{
    /* Initialize data buffer which for storing data from adc */
    for (uint32_t i = 0; i < GDMA_TRANSFER_SIZE; i++)
    {
        ADC_Recv_Data_Buffer[i] = 0;
    }

    /* GDMA init */
    GDMA_InitTypeDef GDMA_InitStruct;
    GDMA_StructInit(&GDMA_InitStruct);

    GDMA_InitStruct.GDMA_ChannelNum         = ADC_GDMA_CHANNEL_NUM;
    GDMA_InitStruct.GDMA_DIR                = GDMA_DIR_PeripheralToMemory;
    GDMA_InitStruct.GDMA_BufferSize         = GDMA_TRANSFER_SIZE;//Determine total transfer size
    GDMA_InitStruct.GDMA_SourceInc          = DMA_SourceInc_Fix;
    GDMA_InitStruct.GDMA_DestinationInc     = DMA_DestinationInc_Inc;
    GDMA_InitStruct.GDMA_SourceDataSize     = GDMA_DataSize_HalfWord;
    GDMA_InitStruct.GDMA_DestinationDataSize = GDMA_DataSize_HalfWord;
    GDMA_InitStruct.GDMA_SourceMsize        = GDMA_Msize_4;
    GDMA_InitStruct.GDMA_DestinationMsize   = GDMA_Msize_4;
    GDMA_InitStruct.GDMA_SourceAddr         = (uint32_t)(&(ADC->FIFO));
    GDMA_InitStruct.GDMA_DestinationAddr    = (uint32_t)ADC_Recv_Data_Buffer;
    GDMA_InitStruct.GDMA_SourceHandshake    = GDMA_Handshake_ADC;

    GDMA_Init(ADC_GDMA_Channel, &GDMA_InitStruct);

    GDMA_INTConfig(ADC_GDMA_CHANNEL_NUM, GDMA_INT_Transfer, ENABLE);

    /* GDMA irq init */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = ADC_GDMA_Channel_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    /* Start to receive data */
    GDMA_Cmd(ADC_GDMA_CHANNEL_NUM, ENABLE);
}

/**
  * @brief  Demo code of operation about adc.
  * @param  No parameter.
  * @return void
*/
void adc_demo(void)
{
//    ADC_CalibrationInit();
    /* Configure pad and pinmux firstly! */
    board_adc_init();
    /* Initialize adc peripheral */
    driver_adc_init();
    /* Initialize timer peripheral */
    driver_adc_timer_init();
    /* Initialize dma peripheral */
    driver_adc_gdma_init();

    ADC_ClearFIFO(ADC);
    ADC_Cmd(ADC, ADC_ONE_SHOT_MODE, ENABLE);
}

/**
  * @brief  Entry of app code
  * @return int (To avoid compile warning)
  */
int main(void)
{
    __enable_irq();
    adc_demo();

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
  * @brief  ADC interrupt handler function.
  * @param  No parameter.
  * @return void
*/
void ADC_Handler(void)
{
    DBG_DIRECT("ADC_Handler!");
    if (ADC_GetINTStatus(ADC, ADC_INT_FIFO_RD_ERR) == SET)
    {
        DBG_DIRECT("ADC_INT_FIFO_RD_ERR");
        ADC_ClearINTPendingBit(ADC, ADC_INT_FIFO_RD_ERR);
        // Add user code here!
    }
    if (ADC_GetINTStatus(ADC, ADC_INT_FIFO_OVERFLOW) == SET)
    {
        DBG_DIRECT("ADC_INT_FIFO_OVERFLOW");
        ADC_ClearINTPendingBit(ADC, ADC_INT_FIFO_OVERFLOW);
        // Add user code here!
    }
}

/**
  * @brief  GDMA channel interrupt handler function.
  * @param  No parameter.
  * @return void
*/
void ADC_GDMA_Channel_Handler(void)
{
    TIM_Cmd(TIM7, DISABLE);
    DBG_DIRECT("ADC_GDMA_Channel_Handler!");
    for (uint32_t i = 0; i < GDMA_TRANSFER_SIZE; i++)
    {
        DBG_DIRECT("ADC_GDMA_Channel_Handler: data[%d] = %d", i, ADC_Recv_Data_Buffer[i]);
    }
//    ADC_ErrorStatus error_status = NO_ERROR;
//    for (uint32_t i = 0; i < GDMA_TRANSFER_SIZE; i++)
//    {
//        DBG_DIRECT("ADC sample data[%d] = %10d          %10.2f", i, ADC_Recv_Data_Buffer[i],
//                   ADC_GetVoltage(DIVIDE_SINGLE_MODE, ADC_Recv_Data_Buffer[i],
//                                  &error_status));
//    }
    GDMA_SetSourceAddress(ADC_GDMA_Channel, (uint32_t)(&(ADC->FIFO)));
    GDMA_SetDestinationAddress(ADC_GDMA_Channel, (uint32_t)(ADC_Recv_Data_Buffer));
    GDMA_SetBufferSize(ADC_GDMA_Channel, GDMA_TRANSFER_SIZE);
    GDMA_ClearINTPendingBit(ADC_GDMA_CHANNEL_NUM, GDMA_INT_Transfer);
    GDMA_Cmd(ADC_GDMA_CHANNEL_NUM, ENABLE);
    ADC_ClearFIFO(ADC);
    TIM_Cmd(TIM7, ENABLE);
}

