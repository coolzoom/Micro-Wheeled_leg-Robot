#include "xbox_dev.h"

xbox gamepad("65:D9:64:B2:08:60");

/**
 * @brief 手柄进程函数
 */
void xbox_dev_proc(uint32_t tick)
{
    gamepad.update();
}
