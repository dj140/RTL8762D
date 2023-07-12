#include <stddef.h>
#include <stdint.h>
#include "rtl876x.h"
#include "system_rtl876x.h"
#include "os_timer.h"
#include "patch_os.h"
#include "trace.h"
#include "core_cm4.h"
#include "core_cmFunc.h"
#include "otp.h"
#include "app_section.h"

#define SYSTEM_CALL_RESTORE_TICK_1MS  0x1001

#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configCPU_CLOCK_HZ                      ( SystemCpuClock )
#define configSYSTICK_CLOCK_HZ                  ( (OTP->systick_clk_src == CORE_CLOCK) ? configCPU_CLOCK_HZ : OTP->systick_ext_clk_freq )
#define portNVIC_SYSTICK_CLK_BIT                ( (OTP->systick_clk_src) << 2UL )
#define SYSTICK_RELOAD_VALUE                    ((configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ) - 1UL)


#define pdFALSE         ( ( BaseType_t ) 0 )
#define pdTRUE          ( ( BaseType_t ) 1 )
#define portMAX_DELAY   ( TickType_t ) 0xffffffffUL

#define portNVIC_SYSTICK_CTRL_REG           ( * ( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG           ( * ( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG  ( * ( ( volatile uint32_t * ) 0xe000e018 ) )

#define portNVIC_SYSTICK_INT_BIT            ( 1UL << 1UL )
#define portNVIC_SYSTICK_ENABLE_BIT         ( 1UL << 0UL )

typedef uint32_t TickType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;
typedef void (*TimerCallbackFunction_t)(void *xTimer);

#define tmrCOMMAND_START                        ( ( BaseType_t ) 1 )
#define tmrCOMMAND_RESET                        ( ( BaseType_t ) 2 )
#define tmrCOMMAND_STOP                         ( ( BaseType_t ) 3 )
#define tmrCOMMAND_CHANGE_PERIOD                ( ( BaseType_t ) 4 )
#define tmrCOMMAND_DELETE                       ( ( BaseType_t ) 5 )

#define queueSEND_TO_BACK       ( ( BaseType_t ) 0 )
#define queueSEND_TO_FRONT      ( ( BaseType_t ) 1 )
#define queueOVERWRITE          ( ( BaseType_t ) 2 )

#define xTimerChangePeriod( xTimer, xNewPeriod, xTicksToWait ) xTimerGenericCommand( ( xTimer ), tmrCOMMAND_CHANGE_PERIOD, ( xNewPeriod ), NULL, ( xTicksToWait ) )
#define xQueueSendToBack( xQueue, pvItemToQueue, xTicksToWait ) xQueueGenericSend( ( xQueue ), ( pvItemToQueue ), ( xTicksToWait ), queueSEND_TO_BACK )
#define xSemaphoreTake( xSemaphore, xBlockTime )        xQueueSemaphoreTake( ( xSemaphore ), ( xBlockTime ) )
#define xSemaphoreTakeRecursive( xMutex, xBlockTime )   xQueueTakeMutexRecursive( ( xMutex ), ( xBlockTime ) )

#define DIVIDE_AND_ROUNDUP(dividend, divisor)   ((dividend + (divisor - 1)) / divisor)

extern uint32_t SystemCpuClock;
extern uint32_t last_lps_remain_us_get(void);

extern void *xTimerCreate(const char *const
                          pcTimerName,            /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
                          const TickType_t xTimerPeriodInTicks,
                          const UBaseType_t uxAutoReload,
                          void *const pvTimerID,
                          TimerCallbackFunction_t pxCallbackFunction);
extern BaseType_t xTimerGenericCommand(void *xTimer, const BaseType_t xCommandID,
                                       const TickType_t xOptionalValue, BaseType_t *const pxHigherPriorityTaskWoken,
                                       const TickType_t xTicksToWait);
extern TickType_t xTaskGetTickCount2(void);
extern void vTaskDelay(const TickType_t xTicksToDelay);
extern BaseType_t xQueueReceive(void *xQueue, void *const pvBuffer, TickType_t xTicksToWait);
extern BaseType_t xQueueGenericSend(void *xQueue, const void *const pvItemToQueue,
                                    TickType_t xTicksToWait, const BaseType_t xCopyPosition);
extern BaseType_t xQueuePeek(void *xQueue, void *const pvBuffer, TickType_t xTicksToWait);
extern BaseType_t xQueueSemaphoreTake(void *xQueue, TickType_t xTicksToWait);
extern BaseType_t xQueueTakeMutexRecursive(void *xMutex, TickType_t xTicksToWait);

static inline bool osif_task_context_check(void)
{
    return (__get_IPSR() == 0);
}

bool os_timer_create_freeRTOS(void **pp_handle, const char *p_timer_name, uint32_t timer_id,
                              uint32_t interval_ms, bool reload, void (*p_timer_callback)(), bool *p_result)
{
    if (pp_handle == NULL || p_timer_callback == NULL)
    {
        *p_result = false;
        return true;
    }

    TickType_t timer_ticks = (TickType_t)(interval_ms);

    if (*pp_handle == NULL)
    {
        *pp_handle = xTimerCreate(p_timer_name, timer_ticks, (BaseType_t)reload,
                                  (void *)timer_id, p_timer_callback);
        if (*pp_handle == NULL)
        {
            *p_result = false;
        }
    }
    else
    {
        *p_result = false;
    }

    return true;
}

bool os_timer_restart_freeRTOS(void **pp_handle, uint32_t interval_ms, bool *p_result)
{
    if (pp_handle == NULL || *pp_handle == NULL)
    {
        *p_result = false;
        return true;
    }

    BaseType_t ret;
    TickType_t timer_ticks = (TickType_t)(interval_ms);

    if (osif_task_context_check() == true)
    {
        ret = xTimerChangePeriod((*pp_handle), timer_ticks, (TickType_t)0);
    }
    else
    {
        return false;  //continue to execute ROM code
    }
    if (ret == pdTRUE)
    {
        *p_result = true;
    }
    else
    {
        *p_result = false;
    }

    return true;
}

bool os_sys_time_get_freeRTOS(uint64_t *p_time_ms)
{
    *p_time_ms = (uint32_t)(xTaskGetTickCount2());
    return true;
}

DATA_RAM_FUNCTION
bool os_timer_next_timeout_value_get_freeRTOS(uint32_t *p_value)
{
    extern uint32_t get_valid_systick_current_value_and_update_tick_count(void);
    uint32_t systick_current_value = get_valid_systick_current_value_and_update_tick_count();

    extern uint32_t lpm_find_nearest_timeout_tick(void);
    uint32_t nearest_timeout_tick = lpm_find_nearest_timeout_tick();

    // xTickCount unit 1msec, wakeup time unit 0.625msec
    if ((nearest_timeout_tick != 0xFFFFFFFF) && (nearest_timeout_tick != 0))
    {
        uint32_t wakeup_time_us = nearest_timeout_tick * 1000 - DIVIDE_AND_ROUNDUP((
                                                                                       SYSTICK_RELOAD_VALUE - systick_current_value) * 1000000,
                                                                                   configSYSTICK_CLOCK_HZ) - last_lps_remain_us_get();

        uint32_t wakeup_time_slot = wakeup_time_us / 625;

        *p_value = (wakeup_time_slot > 0x7D00) ? 0x7D00 : wakeup_time_slot;

        return true;
    }

    *p_value = nearest_timeout_tick;

    return true;
}

bool os_delay_freeRTOS(uint32_t ms)
{
    vTaskDelay((TickType_t)(ms));

    return true;
}

bool os_msg_recv_intern_freeRTOS(void *p_handle, void *p_msg, uint32_t wait_ms,
                                 const char *p_func, uint32_t file_line, bool *p_result)
{
    BaseType_t ret;

    if (osif_task_context_check() == true)
    {
        TickType_t wait_ticks = (TickType_t)(wait_ms);

        ret = xQueueReceive(p_handle, p_msg, wait_ticks);
    }
    else
    {
        return false;  //continue to execute ROM code
    }

    if (ret == pdTRUE)
    {
        *p_result = true;
    }
    else
    {
        *p_result = false;
    }

    return true;
}

bool os_msg_send_intern_freeRTOS(void *p_handle, void *p_msg, uint32_t wait_ms,
                                 const char *p_func, uint32_t file_line, bool *p_result)
{
    BaseType_t ret;
    if (osif_task_context_check() == true)
    {
        TickType_t wait_ticks = (TickType_t)(wait_ms);

        ret = xQueueSendToBack(p_handle, p_msg, wait_ticks);
    }
    else
    {
        return false;  //continue to execute ROM code
    }
    if (ret == pdTRUE)
    {
        *p_result = true;
    }
    else
    {
        *p_result = false;
    }

    return true;
}

bool os_msg_peek_intern_freeRTOS(void *p_handle, void *p_msg, uint32_t wait_ms,
                                 const char *p_func, uint32_t file_line, bool *p_result)
{
    BaseType_t ret;

    if (osif_task_context_check() == true)
    {
        TickType_t wait_ticks = (TickType_t)(wait_ms);

        ret = xQueuePeek(p_handle, p_msg, wait_ticks);
    }
    else
    {
        return false;  //continue to execute ROM code
    }

    if (ret == pdTRUE)
    {
        *p_result = true;
    }
    else
    {
        *p_result = false;
    }

    return true;
}

bool os_sem_take_freeRTOS(void *p_handle, uint32_t wait_ms, bool *p_result)
{
    BaseType_t ret;

    if (osif_task_context_check() == true)
    {
        TickType_t wait_ticks = (TickType_t)(wait_ms);

        ret = xSemaphoreTake(p_handle, wait_ticks);
    }
    else
    {
        return false;  //continue to execute ROM code
    }

    if (ret == pdTRUE)
    {
        *p_result = true;
    }
    else
    {
        *p_result = false;
    }

    return true;
}

bool os_mutex_take_freeRTOS(void *p_handle, uint32_t wait_ms, bool *p_result)
{
    TickType_t wait_ticks = (TickType_t)(wait_ms);
    BaseType_t ret;

    ret = xSemaphoreTakeRecursive(p_handle, wait_ticks);
    if (ret == pdTRUE)
    {
        *p_result = true;
    }
    else
    {
        *p_result = false;
    }

    return true;
}

bool vTaskSwitchContext_app(void)
{
    static bool is_update_systick = false;

    if (is_update_systick == false)
    {
        /* Stop and clear the SysTick. */
        portNVIC_SYSTICK_CTRL_REG = 0UL;
        portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

        /* Configure SysTick to interrupt at the requested rate. */
        portNVIC_SYSTICK_LOAD_REG = (configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ) - 1UL;
        portNVIC_SYSTICK_CTRL_REG = (portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT |
                                     portNVIC_SYSTICK_ENABLE_BIT);

        is_update_systick = true;
    }
    return false;
}

DATA_RAM_FUNCTION
void restore_os_tick_count_1ms(void)
{
    bool ret = false;
    SystemCall(SYSTEM_CALL_RESTORE_TICK_1MS, (uint32_t)&ret);
    if (ret == false)
    {
        APP_PRINT_ERROR0("1ms tick not support");
    }
}

APP_FLASH_TEXT_SECTION
void osif_update_for_freeRTOS(void)
{
    patch_osif_os_timer_create = os_timer_create_freeRTOS;
    patch_osif_os_timer_restart = os_timer_restart_freeRTOS;

    patch_osif_os_sys_time_get = os_sys_time_get_freeRTOS;

    patch_osif_os_delay = os_delay_freeRTOS;
    patch_osif_os_msg_recv_intern = os_msg_recv_intern_freeRTOS;
    patch_osif_os_msg_send_intern = os_msg_send_intern_freeRTOS;
    patch_osif_os_msg_peek_intern = os_msg_peek_intern_freeRTOS;
    patch_osif_os_sem_take = os_sem_take_freeRTOS;
    patch_osif_os_mutex_take = os_mutex_take_freeRTOS;

    //NOTE: patch_vTaskSwitchContext should not be patched in other place
    extern BOOL_PATCH_FUNC patch_vTaskSwitchContext;
    patch_vTaskSwitchContext = vTaskSwitchContext_app;

    //DLPS patch
    OTP->timer_diff_threshold = 10;
    patch_osif_os_timer_next_timeout_value_get = os_timer_next_timeout_value_get_freeRTOS;

    extern void (*restore_os_tick_count)(void);
    restore_os_tick_count = restore_os_tick_count_1ms;
}

USER_CALL_BACK os_patch = osif_update_for_freeRTOS;
