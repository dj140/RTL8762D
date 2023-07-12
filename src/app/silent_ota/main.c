/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     main.c
* @brief    This is the entry of user code which the main function resides in.
* @details
* @author
* @date
* @version
*********************************************************************************************************
*/
#include <stdlib.h>
#include <string.h>
#include "gap.h"
#include "gap_adv.h"
#include "gap_msg.h"
#include "gap_bond_le.h"
#include "gap_conn_le.h"
#include "profile_server.h"
#include "bas.h"
#include "dis.h"
#include "dfu_service.h"
#include "ota_service.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_io_dlps.h"
#include "rtl876x_rcc.h"
#include "rtl876x_nvic.h"
#include "rtl876x_gpio.h"
#include "dlps.h"
#include "os_sched.h"
#include "os_timer.h"
#include "trace.h"
#include "patch_header_check.h"
#include "flash_device.h"
#include "app_task.h"
#include "silent_ota_application.h"
#include "board.h"
#include "otp_config.h"
#include "rtl876x_lib_platform.h"
#if (AON_WDG_ENABLE == 1)
#include "rtl876x_aon_wdg.h"
#endif

/*============================================================================*
 *                              Macros
 *============================================================================*/
/* What is the advertising interval when device is discoverable (units of 625us, 160=100ms)*/
#define DEFAULT_ADVERTISING_INTERVAL_MIN            320 /* 20ms */
#define DEFAULT_ADVERTISING_INTERVAL_MAX            320 /* 20ms */

/*============================================================================*
 *                              External Variables
 *============================================================================*/
T_SERVER_ID g_bas_service_id;
T_SERVER_ID g_dis_service_id;
T_SERVER_ID g_ota_service_id;
T_SERVER_ID g_dfu_service_id;

bool g_allowed_dfu_enter_dlps = true;
uint8_t g_ota_mode;
uint8_t g_keystatus;

#if (AON_WDG_ENABLE == 1)
#define TIMER_WAKEUP_DLPS_PERIOD     ((AON_WDG_TIME_OUT_PERIOD_SECOND - 1) * 1000)
void *xTimerPeriodWakeupDlps;
#endif
/*============================================================================*
 *                              Local Variables
 *============================================================================*/
static uint8_t SCAN_RSP_DATA[] =
{
    /* place holder for Local Name, filled by BT stack. if not present */
    /* BT stack appends Local Name.                                    */
    0x03,                               /* length     */
    GAP_ADTYPE_APPEARANCE,              /* type="Appearance" */
    LO_WORD(GAP_GATT_APPEARANCE_KEYBOARD),
    HI_WORD(GAP_GATT_APPEARANCE_KEYBOARD), /*0xc1, 0x03: Key Board */
};

static uint8_t ADV_DATA[] =
{
    /* Core spec. Vol. 3, Part C, Chapter 18 */
    /* Flags */
    0x02,                               /* length     */
    GAP_ADTYPE_FLAGS,                   /* type="flags" */
    GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,      /*data= "LE General Discoverable Mode", BR/EDR not supp */

    /* Service */
    0x03,                               /* length     */
    GAP_ADTYPE_16BIT_COMPLETE,          /* type="Complete list of 16-bit UUIDs" */
    0x12,
    0x18,

    /* BT stack appends Local Name.                                    */
    0x03,                               /* length     */
    GAP_ADTYPE_APPEARANCE,              /* type="Appearance" */
    LO_WORD(GAP_GATT_APPEARANCE_KEYBOARD),
    HI_WORD(GAP_GATT_APPEARANCE_KEYBOARD), /*0xc1, 0x03: Key Board */

    0x0B,                               /* length     */
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,     /* type="Complete local name" */
    'R', 'e', 'a', 'l', 'T', 'e', 'k', 'D', 'f', 'u',

    0x05,                               /* length     */
    GAP_ADTYPE_MANUFACTURER_SPECIFIC,   /* reserved */
    0x5D,
    0x00,
    0x04,
    0x00,
};
/******************************************************************
 * @fn          Initial gap parameters
 * @brief      Initialize peripheral and gap bond manager related parameters
 *
 * @return     void
 */
void app_le_gap_init(void)
{
    //device name and device appearance
    uint8_t  device_name[GAP_DEVICE_NAME_LEN] = "RealTekDfu";
    uint16_t appearance = GAP_GATT_APPEARANCE_KEYBOARD;
    uint8_t  slave_init_mtu_req = true;

    //advertising parameters
    uint8_t  adv_evt_type = GAP_ADTYPE_ADV_IND;
    uint8_t  adv_direct_type = GAP_REMOTE_ADDR_LE_PUBLIC;
    uint8_t  adv_direct_addr[GAP_BD_ADDR_LEN] = {0};
    uint8_t  adv_chann_map = GAP_ADVCHAN_ALL;
    uint8_t  adv_filter_policy = GAP_ADV_FILTER_ANY;
    uint16_t adv_int_min = DEFAULT_ADVERTISING_INTERVAL_MIN;
    uint16_t adv_int_max = DEFAULT_ADVERTISING_INTERVAL_MIN;

    /* GAP Bond Manager parameters */
    uint8_t  auth_pair_mode = GAP_PAIRING_MODE_PAIRABLE;
    uint16_t auth_flags = GAP_AUTHEN_BIT_BONDING_FLAG;
    uint8_t  auth_io_cap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
#ifndef SUPPORT_ALONE_UPPERSTACK_IMG
    uint8_t  auth_oob = false;
#endif
    uint8_t  auth_use_fix_passkey = false;
    uint32_t auth_fix_passkey = 0;

    uint8_t  auth_sec_req_enable = false;

    uint16_t auth_sec_req_flags = GAP_AUTHEN_BIT_BONDING_FLAG;


    //Register gap callback
    le_register_app_cb(app_gap_callback);

    //Set device name and device appearance
    le_set_gap_param(GAP_PARAM_DEVICE_NAME, GAP_DEVICE_NAME_LEN, device_name);
    le_set_gap_param(GAP_PARAM_APPEARANCE, sizeof(appearance), &appearance);
    le_set_gap_param(GAP_PARAM_SLAVE_INIT_GATT_MTU_REQ, sizeof(slave_init_mtu_req),
                     &slave_init_mtu_req);
    //Set advertising parameters
    le_adv_set_param(GAP_PARAM_ADV_EVENT_TYPE, sizeof(adv_evt_type), &adv_evt_type);
    le_adv_set_param(GAP_PARAM_ADV_DIRECT_ADDR_TYPE, sizeof(adv_direct_type), &adv_direct_type);
    le_adv_set_param(GAP_PARAM_ADV_DIRECT_ADDR, sizeof(adv_direct_addr), adv_direct_addr);
    le_adv_set_param(GAP_PARAM_ADV_CHANNEL_MAP, sizeof(adv_chann_map), &adv_chann_map);
    le_adv_set_param(GAP_PARAM_ADV_FILTER_POLICY, sizeof(adv_filter_policy), &adv_filter_policy);
    le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MIN, sizeof(adv_int_min), &adv_int_min);
    le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MAX, sizeof(adv_int_max), &adv_int_max);
    le_adv_set_param(GAP_PARAM_ADV_DATA, sizeof(ADV_DATA), ADV_DATA);
    le_adv_set_param(GAP_PARAM_SCAN_RSP_DATA, sizeof(SCAN_RSP_DATA), SCAN_RSP_DATA);

    // Setup the GAP Bond Manager
    gap_set_param(GAP_PARAM_BOND_PAIRING_MODE, sizeof(auth_pair_mode), &auth_pair_mode);
    gap_set_param(GAP_PARAM_BOND_AUTHEN_REQUIREMENTS_FLAGS, sizeof(auth_flags), &auth_flags);
    gap_set_param(GAP_PARAM_BOND_IO_CAPABILITIES, sizeof(auth_io_cap), &auth_io_cap);
#ifndef SUPPORT_ALONE_UPPERSTACK_IMG
    gap_set_param(GAP_PARAM_BOND_OOB_ENABLED, sizeof(auth_oob), &auth_oob);
#endif
    le_bond_set_param(GAP_PARAM_BOND_FIXED_PASSKEY, sizeof(auth_fix_passkey), &auth_fix_passkey);
    le_bond_set_param(GAP_PARAM_BOND_FIXED_PASSKEY_ENABLE, sizeof(auth_use_fix_passkey),
                      &auth_use_fix_passkey);
    le_bond_set_param(GAP_PARAM_BOND_SEC_REQ_ENABLE, sizeof(auth_sec_req_enable), &auth_sec_req_enable);
    le_bond_set_param(GAP_PARAM_BOND_SEC_REQ_REQUIREMENT, sizeof(auth_sec_req_flags),
                      &auth_sec_req_flags);

}

/******************************************************************
 * @fn          Initial profile
 * @brief      Add simple profile service and register callbacks
 *
 * @return     void
 */
void app_le_profile_init(void)
{
    server_init(4);
    g_bas_service_id = bas_add_service(app_profile_callback);
    g_dis_service_id = dis_add_service(app_profile_callback);
    g_dfu_service_id = dfu_add_service(app_profile_callback);
    g_ota_service_id = ota_add_service(app_profile_callback);
    server_register_app_cb(app_profile_callback);
}
/**
 * @brief    pinmux configuration
 * @return   void
 */
void pinmux_configuration(void)
{
    Pinmux_Config(TP0, DWGPIO);
    Pinmux_Config(KEY, DWGPIO);
    return;
}
/**
 * @brief    pad configuration
 * @return   void
 */
void pad_configuration(void)
{
    Pad_Config(TP0, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(KEY, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    return;
}

/**
* @brief    driver_init() contains the initialization of peripherals.
*
*               Both new architecture driver and legacy driver initialization method can be used.
*
* @return  void
*/
void driver_init(void)
{
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    GPIO_InitTypeDef Gpio_Struct;
    GPIO_StructInit(&Gpio_Struct);
    Gpio_Struct.GPIO_Pin = GPIO_GetPin(TP0);
    Gpio_Struct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(&Gpio_Struct);
    g_ota_mode = !GPIO_ReadInputDataBit(GPIO_GetPin(TP0));

    Gpio_Struct.GPIO_Pin = GPIO_GetPin(KEY);
    Gpio_Struct.GPIO_Mode = GPIO_Mode_IN;
    Gpio_Struct.GPIO_ITCmd = ENABLE;
    Gpio_Struct.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
    Gpio_Struct.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
    Gpio_Struct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
    Gpio_Struct.GPIO_DebounceTime = 20;
    GPIO_Init(&Gpio_Struct);

    g_keystatus = GPIO_ReadInputDataBit(GPIO_GetPin(KEY));
    GPIO_MaskINTConfig(GPIO_GetPin(KEY), DISABLE);
    GPIO_INTConfig(GPIO_GetPin(KEY), ENABLE);

    NVIC_InitTypeDef NVIC_InitStruct;
    //NVIC_InitStruct.NVIC_IRQChannel = GPIO5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannel = KEY_IRQ;//P2_4
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

void KEY_INT_Handle(void)//P2_4
{
    APP_PRINT_ERROR0("Enter GPIO20_Handler!");
    GPIO_MaskINTConfig(GPIO_GetPin(KEY), ENABLE);
    g_keystatus = GPIO_ReadInputDataBit(GPIO_GetPin(KEY));

    if (g_keystatus == 0)
    {
        GPIO->INTPOLARITY |= GPIO_GetPin(KEY);
    }
    else
    {
        GPIO->INTPOLARITY &= ~GPIO_GetPin(KEY);
    }
    GPIO_ClearINTPendingBit(GPIO_GetPin(KEY));
    GPIO_MaskINTConfig(GPIO_GetPin(KEY), DISABLE);
}

/**
* @brief    Board_Init() contains the initialization of pinmux settings and pad settings.
*
*               All the pinmux settings and pad settings shall be initiated in this function.
*               But if legacy driver is used, the initialization of pinmux setting and pad setting
*               should be peformed with the IO initializing.
*
* @return  void
*/
void board_init(void)
{
    pinmux_configuration();
    pad_configuration();
}


/**
 * @brief    System_Handler
 * @note     system handle to judge which pin is wake source
 * @return   void
 */
void System_Handler(void)
{
    uint8_t tmpVal;
    DBG_DIRECT("System_Handler");

    //APP_PRINT_INFO0("System_Handler");
    NVIC_DisableIRQ(System_IRQn);

    KEY_INT_Handle();//P2_4  edge triggle , can't interrupt after dlps

    // need clear debounce bit here.
    tmpVal = btaon_fast_read_safe(0x2b);
    btaon_fast_write_safe(0x2b, (tmpVal | BIT7));

    NVIC_ClearPendingIRQ(System_IRQn);

}

#if DLPS_EN
/**
 * @brief this function will be called before enter DLPS
 *
 *  set PAD and wakeup pin config for enterring DLPS
 *
 * @param none
 * @return none
 * @retval void
*/
void dfu_enter_dlps_set(void)
{
    Pad_Config(KEY, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(TP0, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
    System_WakeUpDebounceTime(0x8);
    if (g_keystatus)
    {
        System_WakeUpPinEnable(KEY, PAD_WAKEUP_POL_LOW, PAD_WK_DEBOUNCE_ENABLE);
    }
    else
    {
        System_WakeUpPinEnable(KEY, PAD_WAKEUP_POL_HIGH, PAD_WK_DEBOUNCE_ENABLE);
    }

#if (AON_WDG_ENABLE == 1)
    aon_wdg_enable();
#endif
}

/**
 * @brief this function will be called after exit DLPS
 *
 *  set PAD and wakeup pin config for enterring DLPS
 *
 * @param none
 * @return none
 * @retval void
*/
void dfu_exit_dlps_init(void)
{
    Pad_Config(TP0, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(KEY, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);

#if (AON_WDG_ENABLE == 1)
    aon_wdg_disable();
#endif
}

/**
 * @brief DLPS CallBack function
 * @param none
* @return true : allow enter dlps
 * @retval void
*/
bool dlps_dfu_check(void)
{
    return g_allowed_dfu_enter_dlps;
}
#endif

/**
* @brief    PwrMgr_Init() contains the setting about power mode.
*
* @return  void
*/
void pwr_mgr_init(void)
{
#if DLPS_EN
    if (false == dlps_check_cb_reg(dlps_dfu_check))
    {
        DBG_DIRECT("Error: dlps_check_cb_reg(dlps_dfu_check) failed!\n");
    }
    DLPS_IORegUserDlpsEnterCb(dfu_enter_dlps_set);
    DLPS_IORegUserDlpsExitCb(dfu_exit_dlps_init);
    DLPS_IORegister();
    lps_mode_set(LPM_DLPS_MODE);
#endif
}

#if (AON_WDG_ENABLE == 1)
void vTimerPeriodWakeupDlpsCallback(void *pxTimer)
{
    APP_PRINT_INFO0("TimerPeriodWakeupDlps timeout!");
}
#endif


/**
 * @brief    sw_timer_init
 * @note     create sw timer
 * @return   void
 */
void sw_timer_init(void)
{
#if (AON_WDG_ENABLE == 1)
    bool retval = false;
    retval = os_timer_create(&xTimerPeriodWakeupDlps, "xTimerPeriodWakeupDlps",  1, \
                             TIMER_WAKEUP_DLPS_PERIOD, true, vTimerPeriodWakeupDlpsCallback);
    if (!retval)
    {
        APP_PRINT_INFO1("create xTimerPeriodWakeupDlps retval=%d", retval);
    }
    else
    {
        os_timer_start(&xTimerPeriodWakeupDlps);
        APP_PRINT_INFO0("xTimerPeriodWakeupDlps Start!");
    }
#endif
}

/**
* @brief  Task_Init() contains the initialization of all the tasks.
*
*           There are four tasks are initiated.
*           Lowerstack task and upperstack task are used by bluetooth stack.
*           Application task is task which user application code resides in.
*           Emergency task is reserved.
*
* @return  void
*/
void task_init(void)
{
    app_task_init();
}


/**
* @brief  print all images version to check ota whether success.
* @return  void
*/
void print_all_images_version(void)
{
    T_IMG_ID image_id;
    T_IMAGE_VERSION image_version;

    image_id = OTA;
    if (get_active_bank_image_version(image_id, &image_version))
    {
        DBG_DIRECT("image:0x%x,version =0x%x, sub_version:%d.%d.%d.%d", image_id,
                   image_version.ver_info.version,
                   image_version.ver_info.header_sub_version._version_major,
                   image_version.ver_info.header_sub_version._version_minor,
                   image_version.ver_info.header_sub_version._version_revision,
                   image_version.ver_info.header_sub_version._version_reserve);
    }
    else
    {
        DBG_DIRECT("image:0x%x get_active_bank_image_version fail!!!", image_id);
    }
#ifdef SUPPORT_ALONE_UPPERSTACK_IMG
    for (image_id = SecureBoot; image_id < UpperStack; image_id++)
    {
        if (get_active_bank_image_version(image_id, &image_version))
        {
            DBG_DIRECT("image:0x%x,version =0x%x, sub_version:%d.%d.%d.%d", image_id,
                       image_version.ver_info.version,
                       image_version.ver_info.img_sub_version._version_major,
                       image_version.ver_info.img_sub_version._version_minor,
                       image_version.ver_info.img_sub_version._version_revision,
                       image_version.ver_info.img_sub_version._version_reserve);
        }
        else
        {
            DBG_DIRECT("image:0x%x get_active_bank_image_version fail!!!", image_id);
        }
    }

    image_id = UpperStack;
    extern uint32_t get_header_addr_by_img_id_imp(T_IMG_ID image_id);
    T_IMG_HEADER_FORMAT *p_header = (T_IMG_HEADER_FORMAT *)get_header_addr_by_img_id_imp(image_id);
    if (p_header)
    {
        image_version.ver_info.version = p_header->git_ver.ver_info.version;
        DBG_DIRECT("image:0x%x,version =0x%x, sub_version:%d.%d.%d.%d", image_id,
                   image_version.ver_info.version,
                   image_version.ver_info.img_sub_version._version_major,
                   image_version.ver_info.img_sub_version._version_minor,
                   image_version.ver_info.img_sub_version._version_revision,
                   image_version.ver_info.img_sub_version._version_reserve);
    }
    else
    {
        DBG_DIRECT("image:0x%x get_active_bank_image_version fail!!!", image_id);
    }
#else
    for (image_id = SecureBoot; image_id < IMAGE_MAX; image_id++)
    {
        if (get_active_bank_image_version(image_id, &image_version))
        {
            DBG_DIRECT("image:0x%x,version =0x%x, sub_version:%d.%d.%d.%d", image_id,
                       image_version.ver_info.version,
                       image_version.ver_info.img_sub_version._version_major,
                       image_version.ver_info.img_sub_version._version_minor,
                       image_version.ver_info.img_sub_version._version_revision,
                       image_version.ver_info.img_sub_version._version_reserve);
        }
        else
        {
            DBG_DIRECT("image:0x%x get_active_bank_image_version fail!!!", image_id);
        }
    }
#endif


    T_IMG_CTRL_HEADER_FORMAT user_ctrl_header;
    flash_read_locked(flash_get_bank_addr(FLASH_BKP_DATA1), sizeof(T_IMG_CTRL_HEADER_FORMAT),
                      (uint8_t *)&user_ctrl_header);
    DBG_DIRECT("image:0x%x, not_ready=%d",
               user_ctrl_header.image_id, user_ctrl_header.ctrl_flag.flag_value.not_ready);

    if (user_ctrl_header.image_id == IMAGE_USER_DATA &&
        !user_ctrl_header.ctrl_flag.flag_value.not_ready)
    {
        T_IMAGE_VERSION usr_data_version;
        flash_read_locked(flash_get_bank_addr(FLASH_BKP_DATA1) + 0x60, sizeof(T_IMAGE_VERSION),
                          (uint8_t *)&usr_data_version);

        DBG_DIRECT("image:0x%x,version =0x%x, sub_version:%d.%d.%d.%d", user_ctrl_header.image_id,
                   usr_data_version.ver_info.version,
                   usr_data_version.ver_info.img_sub_version._version_major,
                   usr_data_version.ver_info.img_sub_version._version_minor,
                   usr_data_version.ver_info.img_sub_version._version_revision,
                   usr_data_version.ver_info.img_sub_version._version_reserve);
    }
    else
    {
        DBG_DIRECT("user data is not exist!!!");
    }

}


/**
 * @brief    Entry of APP code
 * @return   int (To avoid compile warning)
 */
int main(void)
{
    extern uint32_t random_seed_value;
    srand(random_seed_value);
    DBG_DIRECT("slient ota app main");

#if (ENABLE_SWITCH_TO_90M_CLK == 1)
    /*after switch system clk, must do flash calibration*/
    set_system_clock(SYSTEM_90MHZ);
#endif

#if (ENABLE_FLASH_TRY_4BIT_MODE == 1)
    if (FLASH_SUCCESS == flash_try_high_speed(FLASH_MODE_4BIT))
    {
        DBG_DIRECT("Flash change to 4 BIT Mode SUCCESS!");
    }
#endif

    print_all_images_version();

    board_init();
    le_gap_init(1);
    gap_lib_init();
    app_le_gap_init();
    app_le_profile_init();
    pwr_mgr_init();
    sw_timer_init();
    task_init();
    os_sched_start();

    return 0;
}
