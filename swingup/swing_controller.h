#pragma once
#include <math.h>
#include "cart_motion.h"
#include "cart_mechanics.h"

// Shared by firmware and nonlinear closed-loop simulation. SI units.
namespace swing_control {
constexpr float kGravity = 9.81f;
constexpr float kPi = 3.14159265358979323846f;
inline float wrap(float x) { return atan2f(sinf(x), cosf(x)); }

struct Parameters {
  float leff = 0.166f;              // 33 same-side cycles at 5–20 degrees
  float pw = 7.0f, pz = 0.85f;
  float pc1 = -0.8f, pc2 = -1.2f;
  float vmax = cart_hardware::default_speed; // 1125 RPM = 0.75 m/s
  float amax_s = cart_hardware::default_acceleration; // 22,500 RPM/s
  float amax_b = cart_hardware::default_acceleration; // 15 m/s^2
  float jmax = cart_hardware::default_jerk; // 75,000 RPM/s^2 = 50 m/s^3
  float amax_m = 0.5f, jmax_m = 10.0f;
  float ke = 2.0f, kpx = 30.0f, kdx = 0.5f;
  float phase_soft = 1.0f;          // rad/s scale of smooth phase feedback
  float catch_a = 0.60f, catch_r = 3.0f, giveup = 0.80f;
  float rail = 0.150f, bw = 10.0f, vman = 0.05f;
};
struct Gains { float k1, k2, k3, k4; };
inline Gains gains(const Parameters &p) {
  const float b1 = 2*p.pz*p.pw, b0 = p.pw*p.pw;
  const float c1 = -(p.pc1+p.pc2), c0 = p.pc1*p.pc2;
  Gains k;
  k.k3 = -b0*c0*p.leff/kGravity;
  k.k4 = -(b1*c0+b0*c1)*p.leff/kGravity;
  k.k1 = p.leff*k.k3-kGravity-p.leff*(b0+b1*c1+c0);
  k.k2 = p.leff*(k.k4-b1-c1);
  return k;
}
inline float balance(const Gains &k, float theta, float omega, float x, float v) {
  return -(k.k1*theta+k.k2*omega+k.k3*x+k.k4*v);
}
inline float energy(float theta, float omega, float leff) {
  return 0.5f*leff/kGravity*omega*omega+cosf(theta);
}
inline void estimate(float measured, float bandwidth, float dt, float &angle, float &rate) {
  const float w = 2*kPi*bandwidth;
  const float error = wrap(measured-angle);
  angle = wrap(angle+(rate+2*w*error)*dt);
  rate += w*w*error*dt;
}
struct PumpState {
  float elapsed = 0, quiet = 0;
  void reset() { elapsed = quiet = 0; }
};
inline float swing(const Parameters &p, PumpState &s, float theta, float omega,
                   float x, float v, float dt) {
  s.elapsed += dt;
  // Continuous phase feedback replaces the noisy sign relay. Small sensor
  // motion now produces a small demand, not a full +/- energy-pump reversal.
  const float phase = tanhf(omega*cosf(theta)/p.phase_soft);
  float a = p.ke*(energy(theta, omega, p.leff)-1)*phase-p.kpx*x-p.kdx*v;
  if (fabsf(omega)<0.15f && fabsf(wrap(theta-kPi))<0.08f) s.quiet += dt;
  else s.quiet = 0;
  // Gentle startup bias only in the first 1.5 s, away from the rail ends.
  // No repeated full-acceleration kicks when the motor fails to follow.
  if (s.quiet>0.30f && s.elapsed<1.50f && fabsf(x)<0.10f)
    a = fminf(1.0f, p.amax_s);
  return cart_motion::clamp(a, -p.amax_s, p.amax_s);
}
inline bool canCapture(const Parameters &p, const Gains &k, float theta,
                       float omega, float x, float v, float acceleration=0) {
  const float demand = balance(k, theta, omega, x, v);
  if (fabsf(theta)>=p.catch_a || fabsf(omega)>=p.catch_r ||
      fabsf(x)>=fminf(0.12f,p.rail-0.03f) || fabsf(v)>=0.95f*p.vmax ||
      fabsf(demand)>=p.amax_b) return false;
  // Engage earlier on approach, not while departing the expanded window.
  if (fabsf(theta)>0.20f && theta*omega>0) return false;
  // Estimate the initial motion while the existing acceleration slews toward
  // balance demand. A capped short forecast avoids claiming a full motor model.
  const float ramp = fminf(0.10f, fabsf(demand-acceleration)/p.jmax);
  const float alpha = (kGravity*sinf(theta)-acceleration*cosf(theta))/p.leff;
  const float predicted = theta+omega*ramp+0.5f*alpha*ramp*ramp;
  return fabsf(predicted)<p.giveup;
}

// This observes pendulum response, NOT motor position. It can detect the gross
// "pulses but no pendulum motion" failure seen in the log, not all missed steps.
struct ResponseWatch {
  float elapsed = 0, initial_angle = 0, angle_change = 0, travel = 0;
  bool armed = false;
  void reset(float theta) {
    elapsed = angle_change = travel = 0; initial_angle = theta; armed = true;
  }
  bool fault(float theta, float command_velocity, float dt) {
    if (!armed) return false;
    elapsed += dt;
    angle_change = fmaxf(angle_change, fabsf(wrap(theta-initial_angle)));
    travel += fabsf(command_velocity)*dt;
    if (angle_change>0.03f) { armed = false; return false; }
    return elapsed>2.0f && travel>0.03f;
  }
};
} // namespace swing_control
