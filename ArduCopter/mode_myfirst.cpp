#include "Copter.h"

/*
 * Init and run calls for myfirst flight mode
 */

// myfirst_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeMyfirst::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    static float time = 0;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);
    if (target_pitch > 1000){
    time = 0;
    }

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }
    
    //target_pitch = 2000;
    //printf("target_pitch",target_pitch);
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto: Missing Takeoff Cmd");
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "vx, vy: %f, %f", target_vx, target_vy);
    
    if(time<100.0){
            target_pitch = 20.0*100;
            time = time + 0.0025;
    }else {
        target_pitch = 0.0;
        //time = 0.0;
    }

    gcs().send_text(MAV_SEVERITY_CRITICAL, "target_pitch: %f", target_pitch);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "time: %f", time);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    //attitude_control->set_throttle_out(get_pilot_desired_throttle(),true,g.throttle_filt);
    attitude_control->set_throttle_out(0.33,true,g.throttle_filt);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "target_throttle: %f", get_pilot_desired_throttle());

}
