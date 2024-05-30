#include "Sub.h"


bool ModeStabilize::init(bool ignore_checks) {
    // set target altitude to zero for reporting
    position_control->set_pos_target_z_cm(0);
    ardusub.last_pilot_heading = ahrs.yaw_sensor;

    return true;
    return true;
}

void ModeStabilize::run()
{
  uint32_t tnow = AP_HAL::millis();
    float target_roll, target_pitch;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        ardusub.last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // convert pilot input to lean angles
    // To-Do: convert ardusub.get_pilot_desired_lean_angles to return angles as floats
    // TODO2: move into mode.h
    // ardusub.aparm.angle_max表示最大角度，这个参数在mav.parm文件中设置
    // 根据摇杆输入 确定出期望的倾斜角度
    // ardusub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, ardusub.aparm.angle_max);
    ardusub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, ardusub.aparm.angle_max);

    // get pilot's desired yaw rate
    float yaw_input = channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * ardusub.gain, channel_yaw->get_radio_trim());
    float target_yaw_rate = ardusub.get_pilot_desired_yaw_rate(yaw_input);

    // call attitude controller
    // update attitude controller targets

    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        ardusub.last_pilot_heading = ahrs.yaw_sensor;
        ardusub.last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw maneuvers
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < ardusub.last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0;  // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
            ardusub.last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { // call attitude controller holding absolute absolute bearing
            attitude_control->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, ardusub.last_pilot_heading, true);
        }
    }

    // output pilot's throttle
    attitude_control->set_throttle_out(channel_throttle->norm_input(), false, g.throttle_filt);
    if (k++>=400) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "1-4Motors:   %6.3f   %6.3f   %6.3f   %6.3f", motors._thrust_rpyt_out[0], motors._thrust_rpyt_out[1],motors._thrust_rpyt_out[2],motors._thrust_rpyt_out[3]);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "5-7Motors:   %6.3f   %6.3f   %6.3f   %6.3f", motors._thrust_rpyt_out[4], motors._thrust_rpyt_out[5],motors._thrust_rpyt_out[6],motors._thrust_rpyt_out[7]);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "-------------------------------------------------");
        k=0;
    }
    
    //control_in is range -1000-1000
    //radio_in is raw pwm value
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
