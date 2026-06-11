#include "controller_base.h"
#include "controller.h"
#include "sts3032.h"

controller_base::controller_base(){}

void controller_base::update_feedback_gain(float height)
{
    // LQI 增益仅在标定腿高范围内插值，避免极限位外推导致增益过大/抖动
    const float h = constrain(
        height,
        ctrl->lqi_param.car.leg_min_height,
        ctrl->lqi_param.car.leg_max_height
    );

    if(fabsf(h - last_height) < 1.0e-4f){return;}
    last_height = h;
    const float h2 = h * h;
    const float h3 = h2 * h;
    for(uint8_t i = 0; i < 6; i++)
    {
        ctrl->lqi_param.feedback_gain[0][i] =
            ctrl->lqi_param.gain_poly[i][0] * h3 +
            ctrl->lqi_param.gain_poly[i][1] * h2 +
            ctrl->lqi_param.gain_poly[i][2] * h +
            ctrl->lqi_param.gain_poly[i][3];

        ctrl->lqi_param.feedback_gain[1][i] =
            ctrl->lqi_param.gain_poly[i + 6][0] * h3 +
            ctrl->lqi_param.gain_poly[i + 6][1] * h2 +
            ctrl->lqi_param.gain_poly[i + 6][2] * h +
            ctrl->lqi_param.gain_poly[i + 6][3];
    }
}

float controller_base::leg_servo_count_to_height(void)
{
    if(last_position[0] == sts_servo_state[0].position && last_position[1] == sts_servo_state[1].position){return last_value;}
    for(uint8_t i = 0; i < 2; i++)
    {
        last_position[i] = sts_servo_state[i].position;
    }

    float dCount = (fabsf((float)sts_servo_state[0].position - 2048.0f) + fabsf((float)sts_servo_state[1].position - 2048.0f)) * 0.5f;
    float h = ((4.6289047954e-12f * dCount - 9.3936274976e-08f) * dCount + 1.5357902969e-04f) * dCount + 4.2041568108e-02f;
    last_value = h;

    return h;
}

bool controller_base::is_leg_at_mechanical_limit(void)
{
    const int16_t margin = 10;
    const int16_t left_pos = sts_servo_state[0].position;
    const int16_t right_pos = sts_servo_state[1].position;

    if(left_pos <= SERVO_LEFT_MIN + margin || left_pos >= SERVO_LEFT_MAX - margin)
    {
        return true;
    }

    if(right_pos <= SERVO_RIGHT_MAX + margin || right_pos >= SERVO_RIGHT_MIN - margin)
    {
        return true;
    }

    return false;
}

float controller_base::get_height_limit_blend(float height)
{
    const float h_min = ctrl->lqi_param.car.leg_min_height;
    const float h_max = ctrl->lqi_param.car.leg_max_height;
    const float margin = 0.004f;
    const float min_blend = 0.35f;

    float blend = 1.0f;

    if(height <= h_min)
    {
        blend = min_blend;
    }
    else if(height < h_min + margin)
    {
        blend = min_blend + (1.0f - min_blend) * (height - h_min) / margin;
    }
    else if(height >= h_max)
    {
        blend = min_blend;
    }
    else if(height > h_max - margin)
    {
        blend = min_blend + (1.0f - min_blend) * (h_max - height) / margin;
    }

    if(is_leg_at_mechanical_limit())
    {
        blend = fminf(blend, 0.45f);
    }

    return blend;
}

void controller_base::update_linear_reference(float dt, float target_speed)
{
    const bool jump_active = (ctrl->fsm_state_machine.mode == fsm::mode_state::JUMP);
    const float tau = 0.024f;
    const float max_ref_accel = 1.60f;
    const float max_ref_decel = 3.10f;
    const float max_release_decel = 7.20f;
    const float release_duration = 0.45f;
    const float release_stop_speed = 0.035f;
    const float alpha = 1.0f - expf(-dt / tau);
    const bool zero_cmd = fabsf(target_speed) < ctrl->dead_zone;
    const bool had_cmd = fabsf(last_linear_target_speed) >= ctrl->dead_zone;
    const bool start_release = zero_cmd && had_cmd;
    lpf_target_linear_vel += (target_speed - lpf_target_linear_vel) * alpha;

    if(ctrl->balance_recover_active)
    {
        lpf_target_linear_vel = 0.0f;
        ctrl->lqi_param.ref.linear_vel = 0.0f;
        ctrl->lqi_param.integral.linear_vel_error = 0.0f;
        linear_release_active = 0;
        linear_release_timer = 0.0f;
        last_linear_target_speed = 0.0f;
        return;
    }

    if(jump_active)
    {
        lpf_target_linear_vel = 0.0f;
        ctrl->lqi_param.ref.linear_vel = ctrl->jump_linear_vel_cmd;
        linear_release_active = 0;
        linear_release_timer = 0.0f;
        last_linear_target_speed = 0.0f;
        return;
    }

    if(!zero_cmd)
    {
        linear_release_active = 0;
        linear_release_timer = 0.0f;
    }
    else if(start_release)
    {
        linear_release_active = 1;
        linear_release_timer = 0.0f;
        lpf_target_linear_vel = 0.0f;
    }

    float target_ref =
        (fabsf(lpf_target_linear_vel) < ctrl->dead_zone) ? 0.0f : lpf_target_linear_vel;

    // Keep the velocity integrator intact while a short release window
    // snaps the reference back to zero after the stick is released.
    if(linear_release_active)
    {
        linear_release_timer += dt;
        target_ref = 0.0f;

        if(fabsf(ctrl->lqi_param.state.avg_linear_vel) < release_stop_speed ||
           linear_release_timer >= release_duration)
        {
            linear_release_active = 0;
            linear_release_timer = 0.0f;
            target_ref = 0.0f;
        }
    }

    float delta = target_ref - ctrl->lqi_param.ref.linear_vel;
    float ref_rate = ((fabsf(target_ref) > fabsf(ctrl->lqi_param.ref.linear_vel)) ? max_ref_accel : max_ref_decel);
    if(linear_release_active)
    {
        ref_rate = max_release_decel;
    }
    float max_step = ref_rate * dt;
    ctrl->lqi_param.ref.linear_vel += constrain(delta, -max_step, max_step);
    if(fabsf(target_ref) < ctrl->dead_zone && fabsf(ctrl->lqi_param.ref.linear_vel) < max_step)
    {
        ctrl->lqi_param.ref.linear_vel = 0.0f;
    }

    const bool freeze_linear_integral = linear_release_active;
    if(!freeze_linear_integral)
    {
        ctrl->lqi_param.integral.linear_vel_error +=
            (ctrl->lqi_param.ref.linear_vel - ctrl->lqi_param.state.avg_linear_vel) * dt;
        ctrl->lqi_param.integral.linear_vel_error = constrain(
            ctrl->lqi_param.integral.linear_vel_error,
            -ctrl->lqi_param.integral_clamp.linear_vel_error,
            ctrl->lqi_param.integral_clamp.linear_vel_error
        );
    }

    last_linear_target_speed = zero_cmd ? 0.0f : target_speed;
}

void controller_base::update_yaw_reference(float dt, float target_speed)
{
    const bool jump_active = (ctrl->fsm_state_machine.mode == fsm::mode_state::JUMP);
    const float tau = 0.009f;
    const float alpha = 1.0f - expf(-dt / tau);
    lpf_target_steering_vel += (target_speed - lpf_target_steering_vel) * alpha;

    if(ctrl->balance_recover_active)
    {
        lpf_target_steering_vel = 0.0f;
        ctrl->lqi_param.ref.yaw_rate = 0.0f;
        ctrl->lqi_param.integral.yaw_rate_error = 0.0f;
        return;
    }

    if(jump_active)
    {
        lpf_target_steering_vel = 0.0f;
        ctrl->lqi_param.ref.yaw_rate = ctrl->jump_turn_yaw_rate_cmd;

        const bool jump_yaw_integral_active =
            (ctrl->jump_linear_direction != 0) ||
            (ctrl->jump_turn_direction != 0 &&
             (ctrl->fsm_state_machine.jump == fsm::jump_state::PREPARE ||
              ctrl->fsm_state_machine.jump == fsm::jump_state::PUSH ||
              ctrl->fsm_state_machine.jump == fsm::jump_state::FLY));

        if(jump_yaw_integral_active)
        {
            ctrl->lqi_param.integral.yaw_rate_error +=
                (ctrl->lqi_param.ref.yaw_rate - ctrl->lqi_param.state.yaw_rate) * dt;
            ctrl->lqi_param.integral.yaw_rate_error = constrain(
                ctrl->lqi_param.integral.yaw_rate_error,
                -ctrl->lqi_param.integral_clamp.yaw_rate_error,
                ctrl->lqi_param.integral_clamp.yaw_rate_error
            );
        }
        else
        {
            ctrl->lqi_param.integral.yaw_rate_error = 0.0f;
        }
        return;
    }

    ctrl->lqi_param.ref.yaw_rate = -lpf_target_steering_vel;

    ctrl->lqi_param.integral.yaw_rate_error +=
        (ctrl->lqi_param.ref.yaw_rate - ctrl->lqi_param.state.yaw_rate) * dt;
    ctrl->lqi_param.integral.yaw_rate_error = constrain(
        ctrl->lqi_param.integral.yaw_rate_error,
        -ctrl->lqi_param.integral_clamp.yaw_rate_error,
        ctrl->lqi_param.integral_clamp.yaw_rate_error
    );
}

void controller_base::reset_motion_reference()
{
    lpf_target_linear_vel = 0.0f;
    lpf_target_steering_vel = 0.0f;
    last_linear_target_speed = 0.0f;
    linear_release_timer = 0.0f;
    linear_release_active = 0;

    ctrl->lqi_param.ref.linear_vel = 0.0f;
    ctrl->lqi_param.ref.yaw_rate = 0.0f;

    ctrl->lqi_param.integral.linear_vel_error = 0.0f;
    ctrl->lqi_param.integral.yaw_rate_error = 0.0f;
}

void controller_base::set_cam_angle(uint32_t tick)
{
    float dt = (float)tick * 1.0e-3f;

    float speed = 0.0f;
    if(ctrl->buttons & BTN_SELECT)
    {
        if(ctrl->buttons & BTN_UP){speed = cam_max_speed;}
        else if(ctrl->buttons & BTN_DOWN){speed = -cam_max_speed;}
    }

    const float tau = 0.05f;
    float alpha = 1.0f - expf(-dt / tau);
    cam_lpf_target_speed += (speed - cam_lpf_target_speed) * alpha;

    cam_angle += cam_lpf_target_speed * dt;
    cam_angle = constrain(cam_angle, CAMSERVO_MIN, CAMSERVO_MAX);

    if((int16_t)cam_angle != cam_last_angle)
    {
        cam_last_angle = cam_angle;
        cam_servo.set_angle(cam_angle);
    }
}

void controller_base::reset()
{
    reset_motion_reference();
    ctrl->roll_adjust = 0.0f;
    ctrl->leg_height_base = (float)LEG_HEIGHT_BASE;
    ctrl->pid_roll_angle.reset();
}
