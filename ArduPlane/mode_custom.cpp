#include "mode.h"
#include "Plane.h"

/*
  CUSTOM flight mode (mode number 27) — user PID from trapezoidal I + error D + output clamp.

  Rate loop: desired body rates from sticks (same scaling idea as ACRO, using ACRO_*_RATE params).
  Gains default to the user's fixed-wing rate values (P only); tune CUSTOM_PID_* below or add AP params later.
*/

// PID limits (match user sketch ±500)
static constexpr float CUSTOM_PID_I_LIMIT   = 500.0f;
static constexpr float CUSTOM_PID_OUT_LIMIT = 500.0f;
// Map ±CUSTOM_PID_OUT_LIMIT to aileron/elevator/rudder scaled range (Plane uses ~±4500 centi‑fraction units)
static constexpr float CUSTOM_PID_TO_SURFACE = 4500.0f / CUSTOM_PID_OUT_LIMIT;

// Default rate PIDs (deg/s error → arbitrary PID units); I and D optional
static constexpr float CUSTOM_P_ROLL  = 0.5f;
static constexpr float CUSTOM_I_ROLL  = 0.0f;
static constexpr float CUSTOM_D_ROLL  = 0.0f;
static constexpr float CUSTOM_P_PITCH = 0.5f;
static constexpr float CUSTOM_I_PITCH = 0.0f;
static constexpr float CUSTOM_D_PITCH = 0.0f;
static constexpr float CUSTOM_P_YAW   = 0.5f;
static constexpr float CUSTOM_I_YAW   = 0.0f;
static constexpr float CUSTOM_D_YAW   = 0.0f;

namespace {

// Same structure as user pidFeedback(): P on error, trapezoidal I, D on (Error-PrevError)/dt, clamps.
struct CustomPIDState {
    float prev_error;
    float prev_iterm;

    void reset()
    {
        prev_error = 0.0f;
        prev_iterm = 0.0f;
    }

    float step(float error, float P, float I, float D, float dt)
    {
        const float Pterm = P * error;

        float Iterm = prev_iterm + I * (error + prev_error) * 0.5f * dt;
        Iterm = constrain_float(Iterm, -CUSTOM_PID_I_LIMIT, CUSTOM_PID_I_LIMIT);

        const float dt_safe = MAX(dt, 1.0e-6f);
        const float Dterm = D * (error - prev_error) / dt_safe;

        float PIDOutput = Pterm + Iterm + Dterm;
        PIDOutput = constrain_float(PIDOutput, -CUSTOM_PID_OUT_LIMIT, CUSTOM_PID_OUT_LIMIT);

        prev_error = error;
        prev_iterm = Iterm;
        return PIDOutput;
    }
};

static CustomPIDState roll_pid_state;
static CustomPIDState pitch_pid_state;
static CustomPIDState yaw_pid_state;

}  // namespace

bool ModeCustom::_enter()
{
    roll_pid_state.reset();
    pitch_pid_state.reset();
    yaw_pid_state.reset();
    return true;
}

void ModeCustom::update()
{
    // Rate targets are applied in run() from current RC; nav_* unused for this rate PID path.
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;
}

void ModeCustom::run()
{
    const float dt = plane.G_Dt;

#if HAL_QUADPLANE_ENABLED
    if (!plane.quadplane.use_fw_attitude_controllers()) {
        // VTOL attitude path: keep built-in controllers
        plane.stabilize_roll();
        plane.stabilize_pitch();
        plane.mode_stabilize.stabilize_stick_mixing_direct();
        plane.stabilize_yaw();
        output_pilot_throttle();
        return;
    }
#endif

    if (plane.fly_inverted()) {
        // Inverted: fall back to standard stabilization (avoids rate sign surprises here).
        plane.stabilize_roll();
        plane.stabilize_pitch();
        plane.mode_stabilize.stabilize_stick_mixing_direct();
        plane.stabilize_yaw();
        output_pilot_throttle();
        return;
    }

    output_pilot_throttle();

    const float speed_scaler = plane.get_speed_scaler();

    // Desired rates (deg/s), ACRO-style expo and mission planner ACRO_*_RATE limits
    const float rexpo = plane.roll_in_expo(true);
    const float pexpo = plane.pitch_in_expo(true);
    const float rudd_expo = plane.rudder_in_expo(true);

    const float desired_roll_rate_dps  = (rexpo / SERVO_MAX) * plane.g.acro_roll_rate;
    const float desired_pitch_rate_dps = (pexpo / SERVO_MAX) * plane.g.acro_pitch_rate;
    const float desired_yaw_rate_dps   = (rudd_expo / SERVO_MAX) * plane.g.acro_yaw_rate;

    const Vector3f &gvec = plane.ahrs.get_gyro();
    const float actual_roll_rate_dps  = degrees(gvec.x);
    const float actual_pitch_rate_dps = degrees(gvec.y);
    const float actual_yaw_rate_dps   = degrees(gvec.z);

    const float err_roll  = desired_roll_rate_dps  - actual_roll_rate_dps;
    const float err_pitch = desired_pitch_rate_dps - actual_pitch_rate_dps;
    const float err_yaw   = desired_yaw_rate_dps   - actual_yaw_rate_dps;

    const float roll_out  = roll_pid_state.step(err_roll,  CUSTOM_P_ROLL,  CUSTOM_I_ROLL,  CUSTOM_D_ROLL,  dt);
    const float pitch_out = pitch_pid_state.step(err_pitch, CUSTOM_P_PITCH, CUSTOM_I_PITCH, CUSTOM_D_PITCH, dt);
    const float yaw_out   = yaw_pid_state.step(err_yaw,   CUSTOM_P_YAW,   CUSTOM_I_YAW,   CUSTOM_D_YAW,   dt);

    const float scale = CUSTOM_PID_TO_SURFACE * speed_scaler;

    // Tail‑hold takeoff assist (same special case as stabilize_pitch)
    const int8_t force_elevator = plane.takeoff_tail_hold();
    if (force_elevator != 0) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 45 * force_elevator);
    } else {
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator,
                                        constrain_float(pitch_out * scale, -4500.0f, 4500.0f));
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron,
                                    constrain_float(roll_out * scale, -4500.0f, 4500.0f));

    const float rudder_scaled = constrain_float(yaw_out * scale, -4500.0f, 4500.0f);
    output_rudder_and_steering(rudder_scaled);
}
