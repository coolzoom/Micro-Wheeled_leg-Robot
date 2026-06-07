#include "start.h"

// #define DEBUG_TEST
#ifdef DEBUG_TEST

#include "bus/uart_bus.h"
// #include "mpu6050_dev.h"

static void debug_test_proc(uint32_t tick)
{
    static uint8_t init_flag = 0;
    if(!init_flag && (init_flag = 1, 1))
    {
        // uart_bus_init(&uart0);
    }

    // static uint32_t mpu6050_update_cnt = 0;
    // if((mpu6050_update_cnt += tick) >= 5 && (mpu6050_update_cnt = 0, 1))
    // {
    //     mpu6050_dev.update();
    // }

    // left_motor.move(0.2f);

    static uint32_t printf_cnt = 0;
    if((printf_cnt += tick) >= 50 && (printf_cnt = 0, 1))
    {
        // uart0.write_bytes(&uart0, (const uint8_t *)"Hello, world!\n", strlen("Hello, world!\n"));

        // printf("angle_x: %f\tangle_y: %f\tangle_z: %f\n",
        //     mpu6050_dev.angle[0] / (float)PI * 180.0f, mpu6050_dev.angle[1] / (float)PI * 180.0f, mpu6050_dev.angle[2] / (float)PI * 180.0f);

        printf("pitch_angle: %f\tpitch_rate: %f\tavg_linear_pos: %f\tavg_linear_vel: %f\tyaw_angle: %f\tyaw_rate: %f\n",
            ctrl.lqi_param.state.pitch_angle / (float)PI * 180.0f,
            ctrl.lqi_param.state.pitch_rate / (float)PI * 180.0f,
            ctrl.lqi_param.state.avg_linear_pos,
            ctrl.lqi_param.state.avg_linear_vel,
            ctrl.lqi_param.state.yaw_angle / (float)PI * 180.0f,
            ctrl.lqi_param.state.yaw_rate / (float)PI * 180.0f
        );

        // printf("Buttons: ");
        // for(int8_t i = 15; i >= 0; i--)
        // {
        //     printf("%d", (gamepad.buttons >> i) & 0x1);
        // }
        // printf("\t");
        // printf("joysticks: LX:%.3f\tLY:%.3f\tRX:%.3f\tRY:%.3f\tLT:%.3f\t",
        //     gamepad.axes[0],
        //     gamepad.axes[1],
        //     gamepad.axes[2],
        //     gamepad.axes[3],
        //     gamepad.axes[4],
        //     gamepad.axes[5]
        // );
        // printf("triggers: LT:%.3f\tRT:%.3f\n",
        //     gamepad.axes[4],
        //     gamepad.axes[5]
        // );
    }
}

#endif

/**
 * @brief 任务列表
 */
static void task_list(void)
{
#ifdef DEBUG_TEST
    static task debug_test_task(1, debug_test_proc, 4096, 3, 0);
    debug_test_task.start();
#endif

    static task led_dev_task(1000, led_dev_proc, 2048, 2, 0);
    led_dev_task.start();

    static task xbox_dev_task(20, xbox_dev_proc, 4096, 2, 0);
    xbox_dev_task.start();

    static task host_data_task(1, ctrl.host_data.host_data_update_proc, 4096, 3, 0);
    host_data_task.start();

    static task sensor_update_task(1, ctrl.sensor_update_proc, 4096, 4, 0);
    sensor_update_task.start();

    static task motor_update_task(500, ctrl.motor_update_proc);
    motor_update_task.start();

    static task control_loop_task(1, ctrl.loop_proc, 8192, 5, 1);
    control_loop_task.start();
}

/**
 * @brief 初始化所有模块
 */
void start_init_all(void)
{
    delay(1000);

    gamepad.init();
    ctrl.init();

	task_list();
}
