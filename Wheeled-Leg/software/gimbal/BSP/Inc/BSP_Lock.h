#ifndef BSP_LOCK_H
#define BSP_LOCK_H
#include "main.h"
/* Save/restore also works in IRQ callbacks and nested callers. */
static inline uint32_t BSP_Lock(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    return mask;
}
static inline void BSP_Unlock(uint32_t mask) { __set_PRIMASK(mask); }
#endif

