#include "xbox_dev.h"

xbox gamepad("45:52:C1:2D:C1:1C");

/**
 * @brief 手柄进程函数
 */
void xbox_dev_proc(uint32_t tick)
{
    gamepad.update();
}
