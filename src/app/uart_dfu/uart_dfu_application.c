/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     dataTrans_uart.c
* @brief    Data uart operations for testing profiles.
* @details  Data uart init and print data through data uart.
* @author
* @date     2015-03-19
* @version  v0.1
*********************************************************************************************************
*/


#include "rtl876x_uart.h"
#include "rtl876x_rcc.h"
#include "rtl876x_nvic.h"
#include "uart_dfu_application.h"
#include "uart_dfu.h"
#include "rtl876x.h"
#include "trace.h"
#include "app_msg.h"
#include "os_msg.h"

/* task queues used to send message from UART. */
extern void *evt_queue_handle;
extern void *io_queue_handle;
transfer_info UARTConfigInfo;
ReceiveBufStruct IO_Receive;
void datatrans_send_msg_to_app(T_IO_MSG *p_msg)
{
    uint8_t event = EVENT_IO_TO_APP;

    if (os_msg_send(io_queue_handle, p_msg, 0) == false)
    {
        APP_PRINT_ERROR0("send_msg_to_app fail2");
    }
    else if (os_msg_send(evt_queue_handle, &event, 0) == false)
    {
        APP_PRINT_ERROR0("send_msg_to_app fail");
    }
}

void UART0_Handler(void)
{
    uint32_t int_status;
    uint8_t intr_en = 1;
    T_IO_MSG uart_msg;
//    APP_PRINT_ERROR0("uart interrupt");
    /* read interrupt id */
    int_status = UART_GetIID(UART0);

    if (UART_GetFlagStatus(UART0, UART_FLAG_RX_IDLE) == SET)
    {
        //clear Flag
        UART_INTConfig(UART0, UART_INT_RX_IDLE, DISABLE);
        UART_ClearRxFIFO(UART0);
        uart_msg.type = IO_MSG_TYPE_UART;
        uart_msg.subtype = IO_MSG_UART_RX;
        datatrans_send_msg_to_app(&uart_msg);

        //enable idle interrupt again
        UART_INTConfig(UART0, UART_INT_RX_IDLE, ENABLE);
    }

    /* disable interrupt */
    UART_INTConfig(UART0, UART_INT_RD_AVA | UART_INT_RX_LINE_STS, DISABLE);

    switch (int_status)
    {
    /* rx data valiable */
    case UART_INT_ID_RX_LEVEL_REACH:
        {
            if (((8 + IO_Receive.datalen) > (RECEIVE_BUF_MAX_LENGTH - 100)) &&
                UARTConfigInfo.uart_flowctrl)
            {
                APP_PRINT_INFO0("flow ctrl Rx overrun");
                intr_en = 0;
                uart_msg.type = IO_MSG_TYPE_UART;
                uart_msg.subtype = IO_MSG_UART_RX;
                datatrans_send_msg_to_app(&uart_msg);
                IO_Receive.overflow = 1;
                break;
            }
            if ((IO_Receive.WriteOffset + 8) <= RECEIVE_BUF_MAX_LENGTH)
            {
                UART_ReceiveData(UART0, (IO_Receive.buf + IO_Receive.WriteOffset), 8);
                IO_Receive.WriteOffset += 8;
                if (IO_Receive.WriteOffset == RECEIVE_BUF_MAX_LENGTH)
                {
                    IO_Receive.WriteOffset = 0;
                }

            }
            else
            {
                uint16_t len = 0;
                len = RECEIVE_BUF_MAX_LENGTH - IO_Receive.WriteOffset;
                UART_ReceiveData(UART0, (IO_Receive.buf + IO_Receive.WriteOffset), len);
                IO_Receive.WriteOffset = 0;

                UART_ReceiveData(UART0, IO_Receive.buf, 8 - len);
                IO_Receive.WriteOffset += (8 - len);
            }

            if ((8 + IO_Receive.datalen) > RECEIVE_BUF_MAX_LENGTH)   /* Rx overrun */
            {
                APP_PRINT_INFO1("RxDataStatusUpdate: Rx overrun (%d)", 8 + IO_Receive.datalen);
                IO_Receive.datalen = RECEIVE_BUF_MAX_LENGTH;
                IO_Receive.ReadOffset = IO_Receive.WriteOffset;
            }
            else
            {
                IO_Receive.datalen += 8;         /* update length */
            }

        }
        break;

    /* rx time out */
    case UART_INT_ID_RX_DATA_TIMEOUT:
        {
            uint16_t rx_timeout_len = 0;
            while (UART_GetFlagStatus(UART0, UART_FLAG_RX_DATA_AVA) == SET)
            {
                if (IO_Receive.WriteOffset >= RECEIVE_BUF_MAX_LENGTH)
                {
                    APP_PRINT_INFO0("Receive length lager than MAX LENGTH");
                    IO_Receive.WriteOffset = 0;
                }
                UART_ReceiveData(UART0, IO_Receive.buf + IO_Receive.WriteOffset, 1);
                IO_Receive.WriteOffset++;
                rx_timeout_len++;
            }
            if ((rx_timeout_len + IO_Receive.datalen) > RECEIVE_BUF_MAX_LENGTH)   /* Rx overrun */
            {
                APP_PRINT_INFO1("RxDataStatusUpdate2: Rx overrun (%d)", 8 + IO_Receive.datalen);
                IO_Receive.datalen = RECEIVE_BUF_MAX_LENGTH;
                IO_Receive.ReadOffset = IO_Receive.WriteOffset;
            }
            else
            {
                IO_Receive.datalen += rx_timeout_len;         /* update length */
            }


        }
        break;
    /* receive line status interrupt */
    case UART_INT_ID_LINE_STATUS:
        APP_PRINT_INFO1("Line status error!=%x", UART0->LSR);
        break;

    default:
        break;
    }

    if (intr_en)
    {
        UART_INTConfig(UART0, UART_INT_RD_AVA | UART_INT_RX_LINE_STS, ENABLE);
    }

}


void DataTrans_UARTConfig(void)
{
    UART_InitTypeDef UART_InitStruct;
    UART_StructInit(&UART_InitStruct);
    UARTConfigInfo.baudrate = 9600;
    UARTConfigInfo.uart_flowctrl = false;
    if (UARTConfigInfo.uart_flowctrl)
    {
        UART_InitStruct.UART_HardwareFlowControl = UART_HW_FLOW_CTRL_ENABLE;
    }
    else
    {
        UART_InitStruct.UART_HardwareFlowControl = UART_HW_FLOW_CTRL_DISABLE;
    }

    UART_InitStruct.UART_RxThdLevel = 8;

    switch (UARTConfigInfo.baudrate)
    {
    case 2400:
        UART_InitStruct.UART_Div = 1295;
        UART_InitStruct.UART_Ovsr = 7;
        UART_InitStruct.UART_OvsrAdj = 0x7F7;
        break;
    case 4800:
        UART_InitStruct.UART_Div = 542;
        UART_InitStruct.UART_Ovsr = 10;
        UART_InitStruct.UART_OvsrAdj = 0x24A;
        break;
    case 9600:
        UART_InitStruct.UART_Div = 271;
        UART_InitStruct.UART_Ovsr = 10;
        UART_InitStruct.UART_OvsrAdj = 0x24A;
        break;
    case 19200:
        UART_InitStruct.UART_Div = 165;
        UART_InitStruct.UART_Ovsr = 7;
        UART_InitStruct.UART_OvsrAdj = 0x5AD;
        break;
    case 38400:
        UART_InitStruct.UART_Div = 85;
        UART_InitStruct.UART_Ovsr = 7;
        UART_InitStruct.UART_OvsrAdj = 0x222;
        break;
    case 57600:
        UART_InitStruct.UART_Div = 55;
        UART_InitStruct.UART_Ovsr = 7;
        UART_InitStruct.UART_OvsrAdj = 0x5AD;
        break;
    case 115200:
        UART_InitStruct.UART_Div = 20;
        UART_InitStruct.UART_Ovsr = 12;
        UART_InitStruct.UART_OvsrAdj = 0x252;
        break;
    case 921600:
        UART_InitStruct.UART_Div = 3;
        UART_InitStruct.UART_Ovsr = 9;
        UART_InitStruct.UART_OvsrAdj = 0x2AA;
        break;
    default:
        UART_InitStruct.UART_Div = 271;//9600
        UART_InitStruct.UART_Ovsr = 10;
        UART_InitStruct.UART_OvsrAdj = 0x24A;
        break;
    }

    UART_Init(UART0, &UART_InitStruct);
    /*  enable line status interrupt and rx data avaliable interrupt    */
    UART_INTConfig(UART0, UART_INT_RD_AVA | UART_INT_RX_LINE_STS | UART_INT_RX_IDLE,  ENABLE);
}

/****************************************************************************/
/* UART init                                                                */
/****************************************************************************/
void DataTrans_UARTInit(void)
{
    RCC_PeriphClockCmd(APBPeriph_UART0, APBPeriph_UART0_CLOCK, ENABLE);
    /*  UART Init   */
    DataTrans_UARTConfig();

    /*  Enable UART IRQ  */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = UART0_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    return;
}

/******************************************************************
 * @fn          app_handle_io_msg
 * @brief      All the application events are pre-handled in this function.
 *                All the IO MSGs are sent to this function, Then the event handling function
 *                shall be called according to the MSG type.
 *
 * @param    io_driver_msg_recv  - bee io msg data
 * @return     void
 */
void app_handle_io_msg(T_IO_MSG io_driver_msg_recv)
{
    uint16_t msg_type = io_driver_msg_recv.type;

    switch (msg_type)
    {
    case IO_MSG_TYPE_BT_STATUS:
        {
        }
        break;
    case IO_MSG_TYPE_UART:
        {
            dfu_service_handle();
            APP_PRINT_INFO0("IO_MSG_TYPE_UART");
        }
        break;
    default:
        break;
    }
}
