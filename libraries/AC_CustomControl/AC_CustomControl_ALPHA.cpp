#include "AC_CustomControl_ALPHA.h"
#include <stdio.h>

#if CUSTOMCONTROL_ALPHA_ENABLED

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_ALPHA::var_info[] = {
    // @Param: PARAM1
    // @DisplayName: ALPHA param1
    // @Description: Dumy parameter for ALPHA custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM1", 1, AC_CustomControl_ALPHA, param1, 0.0f),

    // @Param: PARAM2
    // @DisplayName: ALPHA param2
    // @Description: Dumy parameter for ALPHA custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM2", 2, AC_CustomControl_ALPHA, param2, 0.0f),

    // @Param: PARAM3
    // @DisplayName: ALPHA param3
    // @Description: Dumy parameter for ALPHA custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM3", 3, AC_CustomControl_ALPHA, param3, 0.0f),

    AP_GROUPEND
};

// initialize in the constructor
AC_CustomControl_ALPHA::AC_CustomControl_ALPHA(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    AP_Param::setup_object_defaults(this, var_info);

    simulink_controller.initialize();
    printf("ALPHA.cpp_initialize");
}

// update controller
// return roll, pitch, yaw controller output
Vector3f AC_CustomControl_ALPHA::update(void)
{
    // reset controller based on spool state
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            // We are still at the ground. Reset custom controller to avoid
            // build up, ex: integrator
            reset();
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // we are off the ground
            break;
    }
    
    // run custom controller after here
    Quaternion attitude_body, attitude_target;
    //Vector3f _euler_angle_target,_euler_angle_body;
    _ahrs->get_quat_body_to_ned(attitude_body);
    /*attitude_target = attitude_body;
    attitude_target.to_euler(_euler_angle_body.x, _euler_angle_body.y, _euler_angle_body.z);
    _euler_angle_body.z = - _euler_angle_body.z;
    attitude_target.from_euler(_euler_angle_body.x, _euler_angle_body.y, _euler_angle_body.z);
    attitude_body = attitude_target;
    */

    attitude_target = _att_control->get_attitude_target_quat();
    //printf("attitude_target:(%f, %f, %f, %f)\n", attitude_target[0],attitude_target[1],attitude_target[2],attitude_target[3]);
    // reversed yaw
    //attitude_target = attitude_target * Quaternion(0.0f, 0.0f, 0.0f, -1.0f);
    //attitude_target.to_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);
    //printf("euler_angle_target.x:%f\n", _euler_angle_target.x);
    //printf("euler_angle_target.y:%f\n", _euler_angle_target.y);
    //printf("euler_angle_target.z:%f\n", _euler_angle_target.z);
    //printf("reversed attitude target\n");
    //_euler_angle_target.z = - _euler_angle_target.z;
    //printf("euler_angle_target.x:%f\n", _euler_angle_target.x);
    //printf("euler_angle_target.y:%f\n", _euler_angle_target.y);
    //printf("euler_angle_target.z:%f\n", _euler_angle_target.z);
    //attitude_target.from_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);
    //printf("attitude_target:(%f, %f, %f, %f)\n", attitude_target[0],attitude_target[1],attitude_target[2],attitude_target[3]);
    // This vector represents the angular error to rotate the thrust vector using x and y and heading using z
    Vector3f attitude_error;
    float _thrust_angle, _thrust_error_angle;
    _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, _thrust_angle, _thrust_error_angle);

    //attitude_error[0] = attitude_target[0] - attitude_body[0];
    //attitude_error[1] = attitude_target[1] - attitude_body[1];
    //attitude_error[2] = attitude_target[2] - attitude_body[2];
    //attitude_error[3] = attitude_target[3] - attitude_body[3];
    // recalculate ang vel feedforward from attitude target model
    // rotation from the target frame to the body frame
    Quaternion rotation_target_to_body = attitude_body.inverse() * attitude_target;
    // target angle velocity vector in the body frame
    Vector3f ang_vel_body_feedforward = rotation_target_to_body * _att_control->get_attitude_target_ang_vel();
    Vector3f gyro_latest = _ahrs->get_gyro_latest();
    gcs().send_text(MAV_SEVERITY_INFO, "ALPHA custom controller working");

    // '<Root>/attiude_error'
    float arg_attiude_error[3]{attitude_error.x, attitude_error.y, attitude_error.z };

    // '<Root>/rate_ff'
    float arg_rate_ff[3]{ang_vel_body_feedforward.x, ang_vel_body_feedforward.y, ang_vel_body_feedforward.z};

    // '<Root>/rate_meas'
    float arg_rate_meas[3]{ gyro_latest.x, gyro_latest.y, gyro_latest.z};

    // '<Root>/Out1'
    float arg_Out1[3];

    
    printf("ALPHA.cpp_simulink_controller.step\n");
    printf("attitude_error: %f,%f,%f\n",arg_attiude_error[0],arg_attiude_error[1],arg_attiude_error[2]);
    printf("rate_ff: %f,%f,%f \n", arg_rate_ff[0],arg_rate_ff[1],arg_rate_ff[2]);
    printf("rate_meas: %f,%f,%f \n", arg_rate_meas[0],arg_rate_meas[1],arg_rate_meas[2]);
    simulink_controller.step(arg_attiude_error, arg_rate_ff, arg_rate_meas, arg_Out1);
    printf("Out: %f,%f,%f \n",arg_Out1[0],arg_Out1[1],arg_Out1[2]);


    // return what arducopter main controller outputted
    return Vector3f(arg_Out1[0], arg_Out1[1], arg_Out1[2]);
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_ALPHA::reset(void)
{
}

#endif
