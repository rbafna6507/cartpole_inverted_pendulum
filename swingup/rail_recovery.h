#pragma once
#include "cart_motion.h"

// Latched rail recovery. Never changes the position origin or disables drivers
// during recovery. User stop/fault resets it and disables drivers separately.
namespace rail_recovery {
constexpr float edge_margin = 0.020f;
constexpr float center_tolerance = 0.003f;
constexpr float quiet_velocity = 0.010f;
constexpr float quiet_acceleration = 0.10f;
constexpr float return_speed = 0.15f;
constexpr float return_acceleration = 1.5f;
enum Phase { NONE=0, BRAKING, CENTERING };
struct State {
  Phase phase = NONE;
  float elapsed = 0, settled = 0;
  void reset() { phase=NONE; elapsed=settled=0; }
  void begin() { if(phase==NONE){phase=BRAKING;elapsed=settled=0;} }
  bool atEdge(float x,float rail_half) const {
    return fabsf(x)>=rail_half-edge_margin;
  }
  bool timedOut() const { return elapsed>8.0f; }
  // True only after reaching center at rest continuously for 100 ms.
  bool update(float x,const cart_motion::State &motion,float dt) {
    if(phase==NONE)return false;
    elapsed+=dt;
    const bool quiet=fabsf(motion.velocity)<=quiet_velocity &&
                     fabsf(motion.acceleration)<=quiet_acceleration;
    if(phase==BRAKING && quiet)phase=CENTERING;
    if(phase==CENTERING && quiet && fabsf(x)<=center_tolerance)settled+=dt;
    else settled=0;
    if(settled>=0.100f){reset();return true;}
    return false;
  }
  float demand(float x,const cart_motion::State &motion,
               float vmax,float amax,float jerk,float dt) const {
    const float target=phase==CENTERING ?
      cart_motion::clamp(-3.0f*x,-fminf(return_speed,vmax),fminf(return_speed,vmax)) : 0;
    const float cap=phase==CENTERING ? fminf(return_acceleration,amax) : amax;
    return cart_motion::velocityAccel(motion.velocity,target,cap,jerk,dt);
  }
};
}
