/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      dfu_application.c
* @brief     dfu application implementation
* @details   dfu application implementation
* @author    ken_mei
* @date      2017-07-03
* @version   v0.1
* *********************************************************************************************************
*/
#include <trace.h>
#include <string.h>
#include <gap_msg.h>
#include <board.h>
#include <app_msg.h>
#include "os_timer.h"
#include "uart_dfu_application.h"
#include "rtl876x_wdg.h"
#include "rtl876x_uart.h"
#include "trace.h"
#include "gap_msg.h"
#include "flash_device.h"
#include "dfu_api.h"
#include "uart_dfu.h"
#include "patch_header_check.h"
#include "board.h"
#include "flash_adv_cfg.h"
#include "os_sched.h"
#include "mem_config.h"
#include "otp.h"
#include "rtl876x_hw_aes.h"
#include "patch_header_check.h"
#include "dfu_flash.h"
#include "os_sched.h"
#include "app_section.h"
#include "os_sync.h"
#include "platform_utils.h"
#include "hw_aes.h"

#define DFU_TEMP_BUFFER_SIZE 2048

uint8_t *g_pOtaTempBufferHead;
uint8_t TempBufferHead[DFU_TEMP_BUFFER_SIZE];
uint16_t ota_tmp_buf_used_size = 0;
TDFU_CB dfuPara;

void getpacket(uint8_t *packet_buffer, uint16_t bufferlen)
{
    uint32_t flags;
    flags = os_lock();//enter critical section

    if (IO_Receive.ReadOffset + bufferlen <= RECEIVE_BUF_MAX_LENGTH)
    {
        memcpy(packet_buffer, IO_Receive.buf + IO_Receive.ReadOffset, bufferlen);
        IO_Receive.ReadOffset += bufferlen;
        if (IO_Receive.ReadOffset == RECEIVE_BUF_MAX_LENGTH)
        {
            IO_Receive.ReadOffset = 0;
        }
    }
    else
    {
        uint16_t len1 = RECEIVE_BUF_MAX_LENGTH - IO_Receive.ReadOffset;
        memcpy(packet_buffer, IO_Receive.buf + IO_Receive.ReadOffset, len1);
        IO_Receive.ReadOffset = 0;
        memcpy(packet_buffer + len1, IO_Receive.buf + IO_Receive.ReadOffset, bufferlen - len1);
        IO_Receive.ReadOffset += bufferlen - len1;
    }

    IO_Receive.datalen -= bufferlen;
    os_unlock(flags); //exit critical section
}

void uart_send_rsp(uint8_t *buf, uint32_t length)
{
    uint32_t i;
    uint32_t count = length / 16;
    uint32_t remainder = length % 16;
    for (i = 0; i < count; i++)
    {
        UART_SendData(UART0, &buf[16 * i], 16);
        while (UART_GetFlagStatus(UART0, UART_FLAG_TX_EMPTY) != SET);
    }
    /* send left bytes */
    UART_SendData(UART0, &buf[16 * i], remainder);
    while (UART_GetFlagStatus(UART0, UART_FLAG_TX_EMPTY) != SET);
}

uint16_t crc16_check(uint8_t *buf, uint16_t len, uint16_t value)
{

    uint16_t b = 0xA001;
    bool reminder = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        value ^= buf[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            reminder = value % 2;
            value >>= 1;
            if (reminder == 1)
            {
                value ^= b;
            }
        }
    }
    return value;
}
void send_rsp(uint8_t opcode, uint16_t len, uint8_t *payload)
{
    uint8_t rsp_event[20] = {};
    rsp_event[0] = DFU_OPCODE_EVENT;
    rsp_event[1] = opcode;
    rsp_event[2] = DFU_OPCODE_EVENT_HEADER;
    rsp_event[3] = len & 0xff;
    rsp_event[4] = len >> 8;
    memcpy(rsp_event + 5, (uint8_t *)payload, len);
    uint16_t crc_value = crc16_check(rsp_event, len + 5, 0xFFFF);
    rsp_event[len + 5] = crc_value & 0xff;
    rsp_event[len + 6] = crc_value >> 8;
    uart_send_rsp(rsp_event, len + 7);
}
void dfu_service_handle()
{
    uint8_t opcode = 0;
    uint16_t length = 0;
    uint16_t    crc_calc_value = 0;
    uint8_t header_err[3] = {0x01, 0x12, 0x01};
    uint8_t packet_header[5];
    uint8_t crc_buffer[2];
    if (IO_Receive.datalen == 0)
    {
        return;
    }
    getpacket(packet_header, 5);
    opcode = packet_header[1];
    header_err[0] = opcode;
    length = packet_header[4] << 8 | packet_header[3];
    APP_PRINT_INFO2("opcode = 0x%x,Receive length = 0x%x", opcode, length);
    if (length + 2 > IO_Receive.datalen)
    {
        APP_PRINT_INFO1("Waiting data IO_Receive.datalen = %d", IO_Receive.datalen);
        return;
    }

    if (packet_header[0] != 0x03)
    {

        header_err[2] = 0x02;
        send_rsp(opcode, 3, header_err);
        DFU_PRINT_INFO1("start byte err =0x%x \r\n", packet_header[0]);
        IO_Receive.ReadOffset = IO_Receive.WriteOffset;
        IO_Receive.datalen = 0;
        return;
    }
    else if (length + 2 != IO_Receive.datalen)
    {
        header_err[2] = 0x03;
        send_rsp(opcode, 3, header_err);
        DFU_PRINT_INFO2("payload length err! length+2=%d ,IO_Receive.datalen=%d\r\n", length + 2,
                        IO_Receive.datalen);
        IO_Receive.ReadOffset = IO_Receive.WriteOffset;
        IO_Receive.datalen = 0;
        return;

    }
    else if ((opcode >= DFU_OPCODE_MAX || opcode <= DFU_OPCODE_MIN) && packet_header[2] != 0x12)
    {
        header_err[2] = 0x04;
        send_rsp(opcode, 3, header_err);
        DFU_PRINT_INFO2("opcode err opcode=0x%x%x \r\n", packet_header[2], opcode);
        IO_Receive.ReadOffset = IO_Receive.WriteOffset;
        IO_Receive.datalen = 0;
        return;
    }
    getpacket(TempBufferHead + ota_tmp_buf_used_size, length);
    crc_calc_value = crc16_check(packet_header, 5, 0xFFFF);
    crc_calc_value = crc16_check(TempBufferHead + ota_tmp_buf_used_size, length, crc_calc_value);
    crc_calc_value = (crc_calc_value >> 8) | (crc_calc_value & 0xff) << 8;
    APP_PRINT_INFO1("crc_calc_value = 0x%x", crc_calc_value);
    getpacket(crc_buffer, 2);
    uint16_t crc16_value = crc_buffer[0] << 8 | crc_buffer[1];
    if (crc16_value != crc_calc_value)
    {
        APP_PRINT_INFO1("crc16_rev err value = 0x%x", crc16_value);
        header_err[2] = 0x01;
        send_rsp(opcode, 3, header_err);
        IO_Receive.ReadOffset = IO_Receive.WriteOffset;
        IO_Receive.datalen = 0;
        return;
    }

    dfu_service_handle_control_point_req(opcode, length, TempBufferHead);
}

DATA_RAM_FUNCTION void dfu_service_handle_control_point_req(uint8_t opcode, uint16_t length,
                                                            uint8_t *p_value)
{
    T_DFU_CTRL_POINT dfu_control_point;
    uint8_t notif_data[20] = {0};
    uint8_t ic_type;
    uint8_t result = 0;
    uint8_t old_bp_lv = 0x02;
    bool check_result;
    uint8_t status = DFU_SUCCESS;
    APP_PRINT_INFO2("dfu_service_handle_control_point_req: opcode=0x%x, length=%d",
                    opcode, length);

    switch (opcode)
    {
    case DFU_OPCODE_START_DFU:
#if HW_AES_KEY
        if (OTP->ota_with_encryption_data)
        {
            DFU_PRINT_TRACE1("Data before decryped: %b", TRACE_BINARY(16, p_value));
            hw_aes_init(OTP->aes_key, NULL, AES_MODE_ECB, OTP->ota_with_encryption_use_aes256);
            hw_aes_decrypt_16byte(p_value, p_value);
            DFU_PRINT_TRACE1("Data after decryped: %b", TRACE_BINARY(16, p_value));
        }
#endif
        dfu_control_point.p.start_dfu.ic_type = (*p_value);
        p_value += 1;
        dfu_control_point.p.start_dfu.secure_version = (*p_value);
        p_value += 1;
        LE_ARRAY_TO_UINT16(dfu_control_point.p.start_dfu.ctrl_flag.value, p_value);
        p_value += 2;
        LE_ARRAY_TO_UINT16(dfu_control_point.p.start_dfu.signature, p_value);
        p_value += 2;
        LE_ARRAY_TO_UINT16(dfu_control_point.p.start_dfu.crc16, p_value);
        p_value += 2;

        LE_ARRAY_TO_UINT32(dfu_control_point.p.start_dfu.image_length, p_value);

        APP_PRINT_INFO6("DFU_OPCODE_START_DFU: ic_type=0x%x, secure_version=0x%x, ctrl_flag.value=0x%x, signature=0x%x,crc16=0x%x*4Bytes, image_length=0x%x",
                        dfu_control_point.p.start_dfu.ic_type,
                        dfu_control_point.p.start_dfu.secure_version,
                        dfu_control_point.p.start_dfu.ctrl_flag.value,
                        dfu_control_point.p.start_dfu.signature,
                        dfu_control_point.p.start_dfu.crc16,
                        dfu_control_point.p.start_dfu.image_length
                       );
        dfuPara.ic_type = dfu_control_point.p.start_dfu.ic_type;
        dfuPara.ctrl_flag.value = dfu_control_point.p.start_dfu.ctrl_flag.value;
        dfuPara.signature = dfu_control_point.p.start_dfu.signature;
        // p_dfu->version = dfu_control_point.p.start_dfu.version;
        dfuPara.crc16 = dfu_control_point.p.start_dfu.crc16;
        dfuPara.image_length = dfu_control_point.p.start_dfu.image_length;

        dfuPara.image_total_length = dfuPara.image_length + IMG_HEADER_SIZE;
        if (dfuPara.ic_type == 0x09 && dfuPara.signature >= OTA && dfuPara.signature <= AppData2 \
            && dfuPara.image_total_length < OTP->ota_bank0_size)
        {

            status = DFU_SUCCESS;
            send_rsp(opcode, 1, &status);
        }
        else
        {
            status = DFU_INVALID_PARAMETER;
            send_rsp(opcode, 1, &status);
        }

        break;

    case DFU_WRITE_IMAGE:
        result = silence_dfu_service_handle_packet_req(length, p_value);

        notif_data[0] = (dfuPara.nCurOffSet + ota_tmp_buf_used_size) & 0xff;
        notif_data[1] = ((dfuPara.nCurOffSet + ota_tmp_buf_used_size) >> 8) & 0xff;
        notif_data[2] = ((dfuPara.nCurOffSet + ota_tmp_buf_used_size) >> 16) & 0xff;
        notif_data[3] = (dfuPara.nCurOffSet + ota_tmp_buf_used_size) >> 24;
        notif_data[4] = result;
        send_rsp(DFU_WRITE_IMAGE, 5, notif_data);

        break;

    case DFU_VALID_IMAGE:

        if (dfuPara.signature < OTA || dfuPara.signature >= IMAGE_MAX)
        {
            APP_PRINT_INFO1("signature err= 0x%x", dfuPara.signature);
            status = DFU_INVALID_PARAMETER;
            send_rsp(DFU_VALID_IMAGE, 1, &status);
            return;
        }
        APP_PRINT_INFO1("DFU_OPCODE_VALID_FW: signature = 0x%x", dfuPara.signature);
        check_result = dfu_check_checksum(dfuPara.signature);
        DFU_PRINT_INFO1("dfu_act_notify_valid, check_result:%d (1: Success, 0: Fail)", check_result);

        if (check_result)
        {
            status = DFU_SUCCESS;
            send_rsp(DFU_VALID_IMAGE, 1, &status);
        }
        else
        {
            status = DFU_ERR;
            send_rsp(DFU_VALID_IMAGE, 1, &status);
        }
        break;

    case DFU_OPCODE_ACTIVE_IMAGE_RESET:
        /*notify bootloader to reset and use new image*/

        APP_PRINT_INFO0("DFU_OPCODE_ACTIVE_IMAGE_RESET:");
        status = DFU_SUCCESS;
        send_rsp(DFU_OPCODE_ACTIVE_IMAGE_RESET, 1, &status);

        if (flash_sw_protect_unlock_by_addr_locked(0x800000, &old_bp_lv))
        {
            APP_PRINT_INFO1("Unlock success! old_bp_lv=%d", old_bp_lv);
        }
        WDG_SystemReset(RESET_ALL, DFU_SWITCH_TO_OTA_MODE);
        break;


    case DFU_OPCODE_SYSTEM_RESET:
        DFU_PRINT_ERROR0("DFU_OPCODE_SYSTEM_RESET");
        status = DFU_SUCCESS;
        send_rsp(DFU_OPCODE_SYSTEM_RESET, 1, &status);
        //os_delay(2000);
        WDG_SystemReset(RESET_ALL, DFU_ACTIVE_RESET);
        break;

    case DFU_OPCODE_REPORT_TARGET_INFO:

        LE_ARRAY_TO_UINT16(dfuPara.signature, p_value);
        PROFILE_PRINT_INFO1("dfuPara.signature is %x\r\n", dfuPara.signature);

        dfu_report_target_fw_info(dfuPara.signature, &dfuPara.origin_image_version,
                                  (uint32_t *)&dfuPara.nCurOffSet);
        dfu_report_target_ic_type(dfuPara.signature, &ic_type);

        notif_data[0] = ic_type;
        notif_data[1] = APP_BANK;
        notif_data[2] = DFU_TEMP_BUFFER_SIZE & 0xff;
        notif_data[3] = DFU_TEMP_BUFFER_SIZE >> 8;
        notif_data[4] = dfuPara.origin_image_version & 0xff;
        notif_data[5] = (dfuPara.origin_image_version >> 8) & 0xff;
        notif_data[6] = (dfuPara.origin_image_version >> 16) & 0xff;
        notif_data[7] = (dfuPara.origin_image_version >> 24) & 0xff;
        notif_data[8] = DFU_SUCCESS;
        send_rsp(DFU_OPCODE_REPORT_TARGET_INFO, 9, notif_data);
        break;
    default:
        {
            APP_PRINT_INFO1("dfu_service_handle_control_point_req: Unknown Opcode=0x%x",
                            dfu_control_point.opcode
                           );
        }

        break;

    }
}

/**
 * @brief silence_dfu_service_handle_packet_req
 *
 * @param length     data reviewed length.
 * @param p_value    data receive point address.
 * @return None
*/
uint8_t silence_dfu_service_handle_packet_req(uint16_t length, uint8_t *p_value)
{
    uint8_t result = 0;
    APP_PRINT_INFO4("dfu_service_handle_packet_req: length=%d, cur_offset =%d, ota_temp_buf_used_size = %d,image_total_length= %d",
                    length,
                    dfuPara.nCurOffSet,
                    ota_tmp_buf_used_size,
                    dfuPara.image_total_length
                   );

    if (dfuPara.nCurOffSet + ota_tmp_buf_used_size + length > dfuPara.image_total_length)
    {
        APP_PRINT_INFO4("dfu_service_handle_packet_req: p_dfu->cur_offset=%d, ota_temp_buf_used_size =%d, length= %d, image_total_length = %d ",
                        dfuPara.nCurOffSet,
                        ota_tmp_buf_used_size,
                        length,
                        dfuPara.image_total_length
                       );
        result = DFU_LENGTH_ERROR;
        PROFILE_PRINT_INFO0("DFU_LENGTH_ERROR");
    }
    else
    {
#if HW_AES_KEY
        if (length >= 16)
        {
            if (OTP->ota_with_encryption_data)
            {
                uint16_t offset = 0;
                do
                {
                    if ((length - offset) >= 16)
                    {
                        hw_aes_init(OTP->aes_key, NULL, AES_MODE_ECB, OTP->ota_with_encryption_use_aes256);
                        hw_aes_decrypt_16byte(p_value + offset, p_value + offset);
                        offset += 16;
                    }
                    else
                    {
                        break;
                    }
                }
                while (1);
            }
        }
#endif
        ota_tmp_buf_used_size += length;

        if (ota_tmp_buf_used_size == 2048/*DFU_TEMP_BUFFER_SIZE*/ ||
            dfuPara.nCurOffSet + ota_tmp_buf_used_size == dfuPara.image_total_length
           )
        {
            unlock_flash_bp_all();
            if (dfu_update(dfuPara.signature, dfuPara.nCurOffSet, ota_tmp_buf_used_size,
                           (uint32_t *)TempBufferHead) == 0)
            {
                lock_flash_bp();
                dfuPara.nCurOffSet += ota_tmp_buf_used_size;
                ota_tmp_buf_used_size = 0;
            }
            else
            {
                result = DFU_PROG_ERROR;
                ota_tmp_buf_used_size -= length;
            }

        }
    }
    return result;
}

