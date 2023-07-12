/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file
* @brief
* @details
* @author
* @date
* @version
* *********************************************************************************************************
*/

#ifndef _UART_DFU_H_
#define _UART_DFU_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <app_msg.h>
#include <profile_server.h>

#define DFU_OPCODE_MIN                          0x00
#define DFU_OPCODE_START_DFU                    0x01
#define DFU_WRITE_IMAGE                         0x02
#define DFU_VALID_IMAGE                         0x03
#define DFU_OPCODE_ACTIVE_IMAGE_RESET           0x04
#define DFU_OPCODE_SYSTEM_RESET                 0x05
#define DFU_OPCODE_REPORT_TARGET_INFO           0x06
#define DFU_OPCODE_MAX                          0x07


#define DFU_OPCODE_EVENT                        0x04
#define DFU_OPCODE_EVENT_HEADER                 0x12


#define DFU_SUCCESS                             0x00
#define DFU_ERR                                 0x01


#define DFU_CRC_ERROR                           0x01
#define DFU_LENGTH_ERROR                        0x02
#define DFU_PROG_ERROR                          0x03
#define DFU_ERASE_ERROR                         0x04
#define DFU_INVALID_PARAMETER                   0x05


#define HW_AES_KEY                              0


typedef struct _T_START_DFU_PARA
{
    uint8_t ic_type;
    uint8_t secure_version;
    union
    {
        uint16_t value;
        struct
        {
            uint16_t xip: 1; // payload is executed on flash
            uint16_t enc: 1; // all the payload is encrypted
            uint16_t load_when_boot: 1; // load image when boot
            uint16_t enc_load: 1; // encrypt load part or not
            uint16_t enc_key_select: 3; // referenced to ENC_KEY_SELECT
            uint16_t not_ready : 1; //for copy image in ota
            uint16_t not_obsolete : 1; //for copy image in ota
            uint16_t rsvd: 7;
        };
    } ctrl_flag;
    uint16_t signature;
    uint16_t crc16;
    uint32_t image_length;
} T_START_DFU_PARA;

typedef struct _T_PKT_RX_NOTIF_REQ
{
    uint16_t packet_num;
} T_PKT_RX_NOTIF_REQ;


typedef struct _T_DFU_CTRL_POINT
{
    uint8_t opcode;
    union
    {
        T_START_DFU_PARA start_dfu;
        T_PKT_RX_NOTIF_REQ pkt_rx_notify_req;
    } p;
} T_DFU_CTRL_POINT, * P_DFU_CTRL_POINT;

typedef struct
{
    uint32_t origin_image_version;

    uint32_t nCurOffSet;
    uint32_t image_total_length;

    uint8_t ic_type;
    uint8_t secure_version;
    union
    {
        uint16_t value;
        struct
        {
            uint16_t xip: 1; // payload is executed on flash
            uint16_t enc: 1; // all the payload is encrypted
            uint16_t load_when_boot: 1; // load image when boot
            uint16_t enc_load: 1; // encrypt load part or not
            uint16_t enc_key_select: 3; // referenced to ENC_KEY_SELECT
            uint16_t not_ready : 1; //for copy image in ota
            uint16_t not_obsolete : 1; //for copy image in ota
            uint16_t rsvd: 7;
        };
    } ctrl_flag;
    uint16_t signature;
    uint16_t crc16;
    uint32_t image_length;

    bool ota_conn_para_upd_in_progress;
    uint8_t mtu_size;
} TDFU_CB;

void  dfu_service_handle_control_point_req(uint8_t opcode, uint16_t length, uint8_t *p_value);
void dfu_service_handle(void);
uint8_t silence_dfu_service_handle_packet_req(uint16_t length, uint8_t *p_value);

#ifdef __cplusplus
}
#endif

#endif

