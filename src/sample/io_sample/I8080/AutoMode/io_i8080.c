/**
*********************************************************************************************************
*               Copyright(c) 2021, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file     io_i8080.c
* @brief    This file provides demo code of I8080 display in auto mode.
* @details
* @author   yuan
* @date     2021-01-08
* @version  v1.0.0
*********************************************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "io_i8080.h"
#include "lcd_st7796_320.h"
#include "trace.h"
#include "rtl876x_tim.h"

/* IF8080 color */
#define WHITE                               ((uint32_t)0xffff)
#define BLACK                               ((uint32_t)0x0000)
#define BLUE                                ((uint32_t)0x001a)
#define GREEN                               ((uint32_t)0x07e0)
#define RED                                 ((uint32_t)0xf800)
#define YELLOW                              ((uint32_t)0xffe0)
#define BROWN                               ((uint32_t)0xbc40)

/* Picture size */
#define X_SIZE                              ((uint32_t)320)
#define Y_SIZE                              ((uint32_t)320)
#define PICTURE_FRAME_SIZE                  ((uint32_t)(X_SIZE*Y_SIZE*2))

/* GDMA defines ---------------------------------------*/
#define GDMA_SOURCE_DATA_WIDTH              ((uint32_t)4)
#define GDMA_FRAME_SIZE                     4000

#define IF8080_GDMA_Channel_NUM             0
#define IF8080_GDMA_Channel                 GDMA_Channel0
#define IF8080_GDMA_Channel_IRQn            GDMA0_Channel0_IRQn
#define IF8080_GDMA_Channel_Handler         GDMA0_Channel0_Handler

/* Globals -------------------------------------------------------------*/
uint32_t IF8080_Color_Group1[X_SIZE * 8];
uint32_t IF8080_Color_Group2[X_SIZE * 8];
GDMA_LLIDef GDMA_LLIStruct_G3;
volatile uint32_t GDMA_FLAG = 0;

/* Funtions -------------------------------------------------------------*/

/**
  * @brief  Initialize the data picture.
  * @param  No parameter.
  * @return void
*/
void data_picture_init(void)
{
    for (uint32_t i = 0; i < sizeof(IF8080_Color_Group1) / sizeof(uint32_t); i += 4)
    {
        IF8080_Color_Group1[i]   = (RED << 16) + RED;
        IF8080_Color_Group1[i + 1] = 0;
        IF8080_Color_Group1[i + 2] = (RED << 16) + RED;
        IF8080_Color_Group1[i + 3] = 0;

        IF8080_Color_Group2[i]   = (GREEN << 16) + GREEN;
        IF8080_Color_Group2[i + 1] = 0;
        IF8080_Color_Group2[i + 2] = (GREEN << 16) + GREEN;
        IF8080_Color_Group2[i + 3] = 0;
    }
}

/**
  * @brief  Initialization of pinmux settings and pad settings.
  * @param  No parameter.
  * @return void
*/
void board_lcd_init(void)
{
    Pad_Config(LCD_8080_D0, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(LCD_8080_D1, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(LCD_8080_D2, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(LCD_8080_D3, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(LCD_8080_D4, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(LCD_8080_D5, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(LCD_8080_D6, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(LCD_8080_D7, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);

    /* CS */
    Pad_Config(LCD_8080_CS, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    /* DCX */
    Pad_Config(LCD_8080_DCX, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    /* RD */
    Pad_Config(LCD_8080_RD, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    /* WR */
    Pad_Config(LCD_8080_WR, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);

    /* 8080 interface: D0~D7 */
    Pinmux_Config(LCD_8080_D0, IDLE_MODE);
    Pinmux_Config(LCD_8080_D1, IDLE_MODE);
    Pinmux_Config(LCD_8080_D2, IDLE_MODE);
    Pinmux_Config(LCD_8080_D3, IDLE_MODE);
    Pinmux_Config(LCD_8080_D4, IDLE_MODE);
    Pinmux_Config(LCD_8080_D5, IDLE_MODE);
    Pinmux_Config(LCD_8080_D6, IDLE_MODE);
    Pinmux_Config(LCD_8080_D7, IDLE_MODE);

    /* CS */
    Pinmux_Config(LCD_8080_CS, IDLE_MODE);
    /* DCX */
    Pinmux_Config(LCD_8080_DCX, IDLE_MODE);
    /* RD */
    Pinmux_Config(LCD_8080_RD, IDLE_MODE);
    /* WR */
    Pinmux_Config(LCD_8080_WR, IDLE_MODE);

    /* BL */
    Pad_Config(LCD_8080_BL, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pinmux_Config(LCD_8080_BL, timer_pwm4);
}

/**
  * @brief  Initialize IF8080 controller peripheral.
  * @param  No parameter.
  * @return void
*/
void driver_lcd_init(void)
{
    /* Open timer clock */
    RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);
    /* Selet the pin grop */
#if IF8080_PINGROUP_1
    IF8080_PinGroupConfig(IF8080_PinGroup_1);
#else
    IF8080_PinGroupConfig(IF8080_PinGroup_2);
#endif
    /*Initialize tim peripheral */
    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    TIM_StructInit(&TIM_InitStruct);
    TIM_InitStruct.TIM_PWM_En = PWM_ENABLE;
    /* Set period */
    TIM_InitStruct.TIM_Period = 10 - 1 ;
    /* Set PWM high count and low count */
    TIM_InitStruct.TIM_PWM_High_Count = 900 - 1 ;
    TIM_InitStruct.TIM_PWM_Low_Count = 100 - 1 ;
    /* Set timer mode User_define */
    TIM_InitStruct.TIM_Mode = TIM_Mode_UserDefine;
    /* Set timer clock source divider 40, timer pclk = 40M/4 */
    TIM_InitStruct.TIM_SOURCE_DIV = TIM_CLOCK_DIVIDER_4;
    TIM_TimeBaseInit(BL_PWM_TIM, &TIM_InitStruct);
    TIM_Cmd(BL_PWM_TIM, ENABLE);

    /* Open IF8080 clock */
    RCC_PeriphClockCmd(APBPeriph_IF8080, APBPeriph_IF8080_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_IF8080, APBPeriph_IF8080_CLOCK, ENABLE);
    /*Initialize IF8080 peripheral */
    IF8080_InitTypeDef IF8080_InitStruct;
    IF8080_StructInit(&IF8080_InitStruct);
    IF8080_InitStruct.IF8080_ClockDiv          = IF8080_CLOCK_DIV_2;
    IF8080_InitStruct.IF8080_Mode              = IF8080_MODE_MANUAL;
    IF8080_InitStruct.IF8080_AutoModeDirection = IF8080_Auto_Mode_Direction_WRITE;
    IF8080_InitStruct.IF8080_GuardTimeCmd      = IF8080_GUARD_TIME_DISABLE;
    IF8080_InitStruct.IF8080_GuardTime         = IF8080_GUARD_TIME_2T;
    IF8080_InitStruct.IF8080_8BitSwap          = IF8080_8BitSwap_ENABLE;
    IF8080_InitStruct.IF8080_16BitSwap         = IF8080_16BitSwap_DISABLE;
    IF8080_InitStruct.IF8080_TxThr             = 10;
    IF8080_InitStruct.IF8080_TxDMACmd          = IF8080_TX_DMA_ENABLE;
    IF8080_InitStruct.IF8080_VsyncCmd          = IF8080_VSYNC_DISABLE;
    IF8080_InitStruct.IF8080_VsyncPolarity     = IF8080_VSYNC_POLARITY_FALLING;
    IF8080_Init(&IF8080_InitStruct);
}

/**
  * @brief  Config GDMA LLIStruct.
  * @param  No parameter.
  * @return void
*/
void GDMA_Config_LLIStruct(uint32_t g1_addr, uint32_t g2_addr, GDMA_InitTypeDef *GDMA_InitStruct)
{
    const uint32_t step = 160;
    static uint32_t percent = 0;

    if (percent < (X_SIZE - step))
    {
        percent += step;
    }
    else
    {
        percent = step;
    }

    GDMA_LLIDef GDMA_LLIStruct_G1;
    GDMA_LLIDef GDMA_LLIStruct_G2;
    IF8080_GDMALLIOFTTypeDef GDMA_LLIStruct_G1_oft;
    IF8080_GDMALLIOFTTypeDef GDMA_LLIStruct_G2_oft;

    GDMA_LLIStruct_G1_oft.SAR_OFT = 4;
    GDMA_LLIStruct_G1_oft.DAR_OFT = 0;
    GDMA_LLIStruct_G2_oft.SAR_OFT = 4;
    GDMA_LLIStruct_G2_oft.DAR_OFT = 0;

    GDMA_LLIStruct_G1.SAR = (uint32_t)(g1_addr);
    GDMA_LLIStruct_G1.DAR = (uint32_t)(&(IF8080->FIFO));
    GDMA_LLIStruct_G1.LLP = 0;
    /* configure low 32 bit of CTL register */
    GDMA_LLIStruct_G1.CTL_LOW = BIT(0)
                                | (GDMA_InitStruct->GDMA_DestinationDataSize << 1)
                                | (GDMA_InitStruct->GDMA_SourceDataSize << 4)
                                | (GDMA_InitStruct->GDMA_DestinationInc << 7)
                                | (GDMA_InitStruct->GDMA_SourceInc << 9)
                                | (GDMA_InitStruct->GDMA_DestinationMsize << 11)
                                | (GDMA_InitStruct->GDMA_SourceMsize << 14)
                                | (GDMA_InitStruct->GDMA_DIR << 20)
                                | (GDMA_InitStruct->GDMA_Multi_Block_Mode & LLP_SELECTED_BIT);
    /* configure high 32 bit of CTL register */
    GDMA_LLIStruct_G1.CTL_HIGH = (percent * 2) / 4;

    GDMA_LLIStruct_G2.SAR = (uint32_t)(g2_addr);
    GDMA_LLIStruct_G2.DAR = (uint32_t)(&(IF8080->FIFO));
    GDMA_LLIStruct_G2.LLP = 0;
    /* configure low 32 bit of CTL register */
    GDMA_LLIStruct_G2.CTL_LOW = BIT(0)
                                | (GDMA_InitStruct->GDMA_DestinationDataSize << 1)
                                | (GDMA_InitStruct->GDMA_SourceDataSize << 4)
                                | (GDMA_InitStruct->GDMA_DestinationInc << 7)
                                | (GDMA_InitStruct->GDMA_SourceInc << 9)
                                | (GDMA_InitStruct->GDMA_DestinationMsize << 11)
                                | (GDMA_InitStruct->GDMA_SourceMsize << 14)
                                | (GDMA_InitStruct->GDMA_DIR << 20)
                                | (GDMA_InitStruct->GDMA_Multi_Block_Mode & LLP_SELECTED_BIT);
    /* configure high 32 bit of CTL register */
    GDMA_LLIStruct_G2.CTL_HIGH = (2 * (X_SIZE - percent)) / 4;

    GDMA_LLIStruct_G3.LLP = 0;
    GDMA_LLIStruct_G3.SAR = (uint32_t)(g2_addr);
    GDMA_LLIStruct_G3.DAR = (uint32_t)(&(IF8080->FIFO));
    /* configure low 32 bit of CTL register */
    GDMA_LLIStruct_G3.CTL_LOW = BIT(0)
                                | (GDMA_InitStruct->GDMA_DestinationDataSize << 1)
                                | (GDMA_InitStruct->GDMA_SourceDataSize << 4)
                                | (GDMA_InitStruct->GDMA_DestinationInc << 7)
                                | (GDMA_InitStruct->GDMA_SourceInc << 9)
                                | (GDMA_InitStruct->GDMA_DestinationMsize << 11)
                                | (GDMA_InitStruct->GDMA_SourceMsize << 14)
                                | (GDMA_InitStruct->GDMA_DIR << 20);
    /* configure high 32 bit of CTL register */
    GDMA_LLIStruct_G3.CTL_HIGH = (2 * (X_SIZE - percent)) / 4;

    IF8080_GDMALLIConfig((IF8080_GDMALLITypeDef *)(&GDMA_LLIStruct_G1),
                         (IF8080_GDMALLITypeDef *)(&GDMA_LLIStruct_G2),
                         (IF8080_GDMALLIOFTTypeDef *)(&GDMA_LLIStruct_G1_oft),
                         (IF8080_GDMALLIOFTTypeDef *)(&GDMA_LLIStruct_G2_oft),
                         Y_SIZE * 2 - 1,
                         (uint32_t)(&GDMA_LLIStruct_G3));
}

/**
  * @brief  Initialize GDMA peripheral.
  * @param  No parameter.
  * @return void
*/
void driver_gdma_init(uint32_t g1_addr, uint32_t g2_addr)
{
    /* Enable GDMA clock */
    RCC_PeriphClockCmd(APBPeriph_GDMA, APBPeriph_GDMA_CLOCK, ENABLE);
    GDMA_Cmd(IF8080_GDMA_Channel_NUM, DISABLE);

    /* Initialize GDMA peripheral */
    GDMA_InitTypeDef GDMA_InitStruct;
    GDMA_StructInit(&GDMA_InitStruct);
    GDMA_InitStruct.GDMA_ChannelNum          = IF8080_GDMA_Channel_NUM;
    GDMA_InitStruct.GDMA_DIR                 = GDMA_DIR_MemoryToPeripheral;
    GDMA_InitStruct.GDMA_BufferSize          = GDMA_FRAME_SIZE;
    GDMA_InitStruct.GDMA_SourceInc           = DMA_SourceInc_Fix;
    GDMA_InitStruct.GDMA_DestinationInc      = DMA_DestinationInc_Fix;
    GDMA_InitStruct.GDMA_SourceDataSize      = GDMA_DataSize_Word;
    GDMA_InitStruct.GDMA_DestinationDataSize = GDMA_DataSize_Word;
    GDMA_InitStruct.GDMA_SourceMsize         = GDMA_Msize_8;
    GDMA_InitStruct.GDMA_DestinationMsize    = GDMA_Msize_8;
    GDMA_InitStruct.GDMA_SourceAddr          = IF8080_LLI_REG1_GDMA_BASE;
    GDMA_InitStruct.GDMA_DestinationAddr     = (uint32_t)(&(IF8080->FIFO));
    GDMA_InitStruct.GDMA_DestHandshake       = GDMA_Handshake_8080_TX;
    GDMA_InitStruct.GDMA_Multi_Block_En      = 1;
    GDMA_InitStruct.GDMA_Multi_Block_Mode    = LLI_TRANSFER;
    GDMA_InitStruct.GDMA_Multi_Block_Struct  = (uint32_t)IF8080_LLI_REG1_GDMA_BASE;
    GDMA_Init(IF8080_GDMA_Channel, &GDMA_InitStruct);

    GDMA_Config_LLIStruct(g1_addr, g2_addr, &GDMA_InitStruct);
    GDMA_INTConfig(IF8080_GDMA_Channel_NUM, GDMA_INT_Transfer, ENABLE);

    /*  Enable GDMA IRQ  */
    NVIC_ClearPendingIRQ((IRQn_Type)IF8080_GDMA_Channel_IRQn);
    NVIC_SetPriority((IRQn_Type)IF8080_GDMA_Channel_IRQn, 3);
    NVIC_EnableIRQ((IRQn_Type)IF8080_GDMA_Channel_IRQn);
}

/**
  * @brief  Set the lcd blacklight.
  * @param  percent: range of 0-100
  * @return void
*/
void lcd_set_backlight(uint32_t percent)
{
    if (percent)
    {
        if (percent > 100)
        {
            percent = 100;
        }
        TIM_Cmd(BL_PWM_TIM, DISABLE);
        TIM_PWMChangeFreqAndDuty(BL_PWM_TIM, percent * 10, (100 - percent) * 10);
        TIM_Cmd(BL_PWM_TIM, ENABLE);
        Pad_Config(LCD_8080_BL, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
                   PAD_OUT_HIGH);
    }
    else
    {
        TIM_Cmd(BL_PWM_TIM, DISABLE);
        Pad_Config(LCD_8080_BL, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
    }
    return;
}

/**
  * @brief  Send command.
  * @param  cmd: command index.
  * @return void
*/
void WriteComm(uint8_t cmd)
{
    IF8080_SetCS();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    IF8080_ResetCS();
    IF8080_SendCommand(cmd);
}

/**
  * @brief  Send data.
  * @param  data: data to be sent.
  * @return void
*/
void WriteData(uint8_t data)
{
    IF8080_SendData(&data, 1);
}

/**
  * @brief  Configure parameter of block write.
  * @param  xStart: start position of X axis.
  * @param  xEnd: end position of X axis.
  * @param  yStart: start position of Y axis.
  * @param  yEnd: end position of Y axis.
  * @return void
*/
void WriteBlock(uint16_t xStart, uint16_t xEnd, uint16_t yStart, uint16_t yEnd)
{
    WriteComm(0x2a);
    WriteData(xStart >> 8);
    WriteData(xStart & 0xff);
    WriteData(xEnd >> 8);
    WriteData(xEnd & 0xff);

    WriteComm(0x2b);
    WriteData(yStart >> 8);
    WriteData(yStart & 0xff);
    WriteData(yEnd >> 8);
    WriteData(yEnd & 0xff);
    IF8080_SetCS();
}

/**
  * @brief  Configure parameter of block write.
  * @param  xStart: start position of X axis.
  * @param  yStart: start position of Y axis.
  * @param  xWidth: width of X axis.
  * @param  yWidth: width of Y axis.
  * @param  color: color data.
  * @return void
*/
void WriteColorBox(uint16_t xStart, uint16_t yStart, uint16_t xWidth, uint16_t yWidth,
                   uint16_t color)
{
    uint32_t temp = 0;

    WriteBlock(xStart, xStart + xWidth - 1, yStart, yStart + yWidth - 1);

    WriteComm(0x2c);
    for (temp = 0; temp < xWidth * yWidth; temp++)
    {
        WriteData(color >> 8);
        WriteData(color);
    }
    IF8080_SetCS();
}

/**
  * @brief  Set delay time.
  * @param  t: time to be set.
  * @return void
*/
void i8080_delay_ms(uint32_t t)
{
    uint32_t SystemCpuClock = 40000000;
    t *= (SystemCpuClock / 10000);

    for (uint32_t i = 0; i < t; ++i)
    {
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
    }
}

/**
  * @brief  Software delay.
  * @param  nCount: delay count.
  * @return void
*/
void delay(uint32_t nCount)
{
    i8080_delay_ms(nCount);
}

/**
  * @brief  Lcd set and reset.
  * @param  reset: the state of the LCD.
  * @return void
*/
void lcd_set_reset(bool reset)
{
    if (reset)
    {
        Pad_Config(LCD_8080_RST, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
    }
    else
    {
        Pad_Config(LCD_8080_RST, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);
    }
}

/**
  * @brief  Write data by auto mode.
  * @param  No parameter.
  * @return void
*/
void lcd_auto_write(uint8_t cmd, uint32_t pixel_num)
{
    /* Enable Auto mode */
    IF8080_SwitchMode(IF8080_MODE_AUTO);

    /* Stop output */
    IF8080_AutoModeCmd(IF8080_Auto_Mode_Direction_WRITE, DISABLE);

    IF8080_ClearTxCounter();

    /* Configure command */
    IF8080_SetCmdSequence(&cmd, 1);

    /* Enable GDMA TX */
    IF8080_GDMACmd(ENABLE);

    /* Configure pixel number */
    IF8080_SetTxDataLen(pixel_num);

    /* Enable GDMA */
    GDMA_Cmd(IF8080_GDMA_Channel_NUM, ENABLE);

    /* Start output */
    IF8080_AutoModeCmd(IF8080_Auto_Mode_Direction_WRITE, ENABLE);
}

/**
  * @brief  Write data by vsync mode.
  * @param  No parameter.
  * @return void
*/
void lcd_auto_write_by_vsync(uint8_t cmd, uint32_t pixel_num)
{
    /* Enable Auto mode */
    IF8080_SwitchMode(IF8080_MODE_AUTO);

    /* Stop output */
    IF8080_AutoModeCmd(IF8080_Auto_Mode_Direction_WRITE, DISABLE);

    IF8080_ClearTxCounter();

    /* Configure command */
    IF8080_SetCmdSequence(&cmd, 1);

    /* Enable GDMA TX */
    IF8080_GDMACmd(ENABLE);

    /* Configure pixel number */
    IF8080_SetTxDataLen(pixel_num);

    /* Enable GDMA */
    GDMA_Cmd(IF8080_GDMA_Channel_NUM, ENABLE);

    /* Start output */
    IF8080_VsyncCmd(ENABLE);
}

/**
  * @brief  Initialize the reset pin .
  * @param  No parameter.
  * @return void
*/
void lcd_reset_init(void)
{
    lcd_set_reset(true);
    platform_delay_ms(50);
    lcd_set_reset(false);
    platform_delay_ms(50);
}

/**
  * @brief  Demo code of IF8080 controller communication.
  * @param  No parameter.
  * @return void
*/
void lcd_write_trigger_by_vsync_demo(void)
{
    /*Initialization of pinmux settings and pad settings */
    board_lcd_init();
    /* Initialize the IF8080 peripheral */
    driver_lcd_init();
    /* Initialize the RST pin */
    lcd_reset_init();
    /* Initialize lcd */
    lcd_st7796_init();
    /* Turn on the LCD power */
    lcd_st7796_power_on();

    while (1)
    {
        /* Enable Manual mode */
        IF8080_SwitchMode(IF8080_MODE_MANUAL);
        WriteBlock(0, X_SIZE - 1, 0, Y_SIZE - 1);
        /* Auto mode operation */
        IF8080_SwitchMode(IF8080_MODE_AUTO);
        /* Initialize the gdma */
        driver_gdma_init((uint32_t)(IF8080_Color_Group1), (uint32_t)(IF8080_Color_Group1));
        /* Write data by vsync mode */
        lcd_auto_write_by_vsync(0x2C, (uint32_t)(PICTURE_FRAME_SIZE));
        while (GDMA_GetChannelStatus(IF8080_GDMA_Channel_NUM) == SET)
        {
            __nop();
            __nop();
            __nop();
            __nop();
            __nop();
            __nop();
            __nop();
            __nop();
            __nop();
            __nop();
            __nop();
            __nop();
        }
    }
}

/**
  * @brief  Demo code of IF8080 controller communication.
  * @param  No parameter.
  * @return void
*/
void lcd_auto_write_by_gdma_demo(void)
{
    /*Initialization of pinmux settings and pad settings */
    board_lcd_init();
    /* Initialize the IF8080 peripheral */
    driver_lcd_init();
    /* Initialize the RST pin */
    lcd_reset_init();
    /* Initialize lcd */
    lcd_st7796_init();
    /* Turn on the LCD power */
    lcd_st7796_power_on();

    while (1)
    {
        /* Enable Manual mode */
        IF8080_SwitchMode(IF8080_MODE_MANUAL);
        WriteBlock(0, X_SIZE - 1, 0, Y_SIZE - 1);
        /* Auto mode operation */
        IF8080_SwitchMode(IF8080_MODE_AUTO);
        /* Initialize the gdma */
        driver_gdma_init((uint32_t)(IF8080_Color_Group1), (uint32_t)(IF8080_Color_Group1));
        /* Write data by auto mode */
        lcd_auto_write(0x2C, (uint32_t)(PICTURE_FRAME_SIZE));
        while (GDMA_GetTransferINTStatus(IF8080_GDMA_Channel_NUM) == RESET)
        {
            __nop();
            __nop();
            __nop();
            __nop();
            __nop();
            __nop();
            __nop();
            __nop();
            __nop();
            __nop();
            __nop();
            __nop();
        }
    }
}

/**
  * @brief  Demo code of i8080.
  * @param  No parameter.
  * @return void
  */
void i8080_demo(void)
{
    DBG_DIRECT("i8080_demo");
    /* Initialize the data picture */
    data_picture_init();
    /* Demo code of IF8080 controller communication */
#if I8080_GDMA_AUTO_MODE_DEMO
    lcd_write_trigger_by_vsync_demo();
#elif I8080_VSYNC_DEMO
    lcd_auto_write_by_gdma_demo();
#endif
}

/**
  * @brief  GDMA interrupt handler function.
  * @param  No parameter.
  * @return void
*/
void GDMA0_Channel0_Handler(void)
{

    GDMA_ClearINTPendingBit(IF8080_GDMA_Channel_NUM, GDMA_INT_Transfer);
    DBG_DIRECT("GDMA0_Channel0_Handler!");
    delay(500);
    GDMA_ClearAllTypeINT(IF8080_GDMA_Channel_NUM);
    /* Enable Manual mode */
    IF8080_SwitchMode(IF8080_MODE_MANUAL);
    WriteBlock(0, X_SIZE - 1, 0, Y_SIZE - 1);
    /* Auto mode operation */
    IF8080_SwitchMode(IF8080_MODE_AUTO);
    /* Initialize the gdma */
    if (GDMA_FLAG == 0)
    {
        driver_gdma_init((uint32_t)(IF8080_Color_Group2), (uint32_t)(IF8080_Color_Group2));
        GDMA_FLAG = 1;
    }
    else
    {
        driver_gdma_init((uint32_t)(IF8080_Color_Group1), (uint32_t)(IF8080_Color_Group1));
        GDMA_FLAG = 0;
    }
    /* Demo code of IF8080 controller communication */
#if I8080_GDMA_DEMO
    lcd_auto_write_by_vsync(0x2C, (uint32_t)(PICTURE_FRAME_SIZE));
#elif I8080_VSYNC_DEMO
    lcd_auto_write(0x2C, (uint32_t)(PICTURE_FRAME_SIZE));
#endif

}

/******************* (C) COPYRIGHT 2021 Realtek Semiconductor Corporation *****END OF FILE****/

