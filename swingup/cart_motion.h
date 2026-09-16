#pragma once
#include <math.h>

// SI units throughout. Kept independent of Arduino so the exact motion math
// can be exercised on the host without driving hardware.
namespace cart_motion {
inline float clamp(float x, float lo, float hi) {
  return fminf(hi, fmaxf(lo, x));
}
inline float slew(float current, float target, float delta) {
  return current + clamp(target - current, -delta, delta);
}

// Acceleration target for approaching a velocity without stepping acceleration
// to zero at arrival. Reserve one control tick in the jerk ramp-down distance.
inline float velocityAccel(float velocity, float target, float amax,
                           float jerk, float dt) {
  const float error = target - velocity;
  const float jd = jerk * dt;
  const float a = fminf(amax, sqrtf(jd*jd + 2*jerk*fabsf(error)) - jd);
  return error >= 0 ? a : -a;
}

// Distance until outward velocity first reaches zero, ramping acceleration
// down at -jerk, then holding -amax if necessary. Includes initial outward
// acceleration, which the old v^2/(2*a) envelope omitted.
inline float stoppingDistance(float velocity, float acceleration,
                              float amax, float jerk) {
  const float v = fmaxf(0, velocity);
  const float a = clamp(acceleration, -amax, amax);
  const float ramp = (a + amax) / jerk;
  const float stop = (a + sqrtf(a*a + 2*jerk*v)) / jerk;
  const float t = fminf(ramp, stop);
  float distance = v*t + 0.5f*a*t*t - jerk*t*t*t/6;
  if (stop > ramp) {
    const float remaining_v = v + a*t - 0.5f*jerk*t*t;
    distance += remaining_v*remaining_v / (2*amax);
  }
  return fmaxf(0, distance);
}

struct State {
  float velocity = 0;
  float acceleration = 0;
  int brake_direction = 0;
};

inline State advance(State s, float x, float requested_accel,
                     float vmax, float drive_amax, float brake_amax,
                     float jerk, float rail_half, float dt) {
  // Leave 10 mm to the physical end for the independent fault stop, plus
  // another 5 mm for step quantization and control timing in normal braking.
  const float boundary = rail_half - 0.015f;
  const float jd = jerk*dt;
  float target = clamp(requested_accel, -drive_amax, drive_amax);
  const float positive = fmaxf(0, velocityAccel(s.velocity, vmax, brake_amax, jerk, dt));
  const float negative = fminf(0, velocityAccel(s.velocity, -vmax, brake_amax, jerk, dt));
  target = clamp(target, negative, positive);
  const float next_a = slew(s.acceleration, target, jd);
  const float next_v = s.velocity + next_a*dt;

  if (s.brake_direction && s.velocity*s.brake_direction <= 0 &&
      s.acceleration*s.brake_direction <= 0)
    s.brake_direction = 0;

  // Look ahead one tick before accepting an acceleration command. Test BOTH
  // rail ends; this also covers accelerating outward from rest near an end.
  if (!s.brake_direction) {
    for (int direction = -1; direction <= 1; direction += 2) {
      const float outward_v = next_v*direction;
      const float outward_a = next_a*direction;
      const float room = boundary - direction*x;
      const float travel = fmaxf(0, outward_v)*dt;
      if ((outward_v > 0 || outward_a > 0) &&
          travel + stoppingDistance(outward_v, outward_a, brake_amax, jerk) >= room) {
        s.brake_direction = direction;
        break;
      }
    }
  }
  if (s.brake_direction) target = -s.brake_direction*brake_amax;
  // Account for velocity gained while acceleration ramps back to zero.
  // Without this anticipation, low-jerk reversals can overshoot vmax even
  // when the requested acceleration has already changed direction.
  const float candidate_a = slew(s.acceleration, target, jd);
  const float candidate_v = s.velocity + candidate_a*dt;
  if (candidate_a > 0 && candidate_v + candidate_a*candidate_a/(2*jerk) >= vmax)
    target = fminf(target, 0.0f);
  if (candidate_a < 0 && candidate_v - candidate_a*candidate_a/(2*jerk) <= -vmax)
    target = fmaxf(target, 0.0f);
  s.acceleration = slew(s.acceleration, target, jd);
  s.velocity += s.acceleration*dt;
  return s;
}
}  // namespace cart_motion
