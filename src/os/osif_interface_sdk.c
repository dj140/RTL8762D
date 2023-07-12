#include <stdlib.h>
#include <string.h>
#include "patch_os.h"
#include "os_task.h"

typedef struct tskTaskControlBlock
{
    uint32_t *pxTopOfStack;

    uint32_t xGenericListItem[5];
    uint32_t xEventListItem[5];
    uint32_t uxPriority;
    uint32_t *pxStack;
    char pcTaskName[10];
} TCB_t;

typedef struct tskTaskControlBlock *TaskHandle_t;

BOOL_PATCH_FUNC patch_osif_os_task_name_get;

bool os_task_name_get(void *p_handle, char **p_task_name)
{
    if (patch_osif_os_task_name_get)
    {
        bool ret;

        if (patch_osif_os_task_name_get(p_handle, p_task_name, &ret))
        {
            return ret;
        }
    }

    TaskHandle_t obj = NULL;

    if (p_handle == NULL)
    {
        os_task_handle_get((void **)&obj);
    }
    else
    {
        obj = (TaskHandle_t)p_handle;
    }

    *p_task_name = &(obj->pcTaskName[0]);

    return true;
}
