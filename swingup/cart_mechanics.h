#pragma once

// Shared pulse/drive geometry. This ceiling describes pulse generation,
// not the motor's measured torque or ability to follow the commands.
namespace cart_hardware {
constexpr int motor_steps = 200, microsteps = 16, pulley_teeth = 20;
constexpr float belt_pitch_mm = 2.0f;
constexpr int isr_hz = 125000;
constexpr float mm_per_rev = pulley_teeth * belt_pitch_mm;
// User-tested motor limits with the 20T pulley (2026-09-16).
constexpr float tested_speed_rpm = 1125.0f;
constexpr float tested_acceleration_rpm_s = 22500.0f;
constexpr float default_speed = tested_speed_rpm * mm_per_rev / 60000.0f;
constexpr float default_acceleration = tested_acceleration_rpm_s * mm_per_rev / 60000.0f;
constexpr float requested_jerk_rpm_s2 = 75000.0f;
constexpr float default_jerk = requested_jerk_rpm_s2 * mm_per_rev / 60000.0f;
constexpr float steps_per_m = motor_steps * microsteps * 1000.0f / mm_per_rev;
constexpr float max_speed = (isr_hz * 0.5f) * mm_per_rev /
                            (motor_steps * microsteps * 1000.0f);
constexpr float max_rpm = isr_hz * 0.5f * 60.0f / (motor_steps * microsteps);
}
