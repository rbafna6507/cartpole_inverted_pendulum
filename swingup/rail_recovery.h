#pragma once
#include "cart_motion.h"

// Latched edge recovery; preserves the existing position origin.
namespace rail_recovery {
constexpr float edge_margin = 0.015f;       // ±135 mm on a 300 mm rail
constexpr float inside_hysteresis = 0.005f; // re-enter by 5 mm before releasing
constexpr float return_speed = 0.15f;
constexpr float return_acceleration = 1.5f;
enum Phase { NONE=0, BRAKING, RETURNING };
struct State {
  Phase phase = NONE;
  float elapsed = 0, exit_boundary = 0;
  int side = 0;
  void reset() { phase=NONE; elapsed=exit_boundary=0; side=0; }
  void begin(float x,float rail_half,int brake_direction=0) {
    if(phase!=NONE)return;
    phase=BRAKING;elapsed=0;
    side=brake_direction?brake_direction:(x>=0?1:-1);
    exit_boundary=rail_half-edge_margin-inside_hysteresis;
  }
  bool atEdge(float x,float rail_half) const { return fabsf(x)>=rail_half-edge_margin; }
  bool timedOut() const { return elapsed>8.0f; }
  bool update(float x,const cart_motion::State &motion,float dt) {
    if(phase==NONE)return false;
    elapsed+=dt;
    // Once outward travel has stopped, direct motion back into the usable
    // range. No center-seeking or dwell: release once safely inside and
    // moving inward (or nearly stopped with inward acceleration).
    const bool inward=motion.velocity*side < -0.005f ||
        (motion.velocity*side<=0.005f && motion.acceleration*side<=0);
    if(phase==BRAKING && inward)phase=RETURNING;
    if(phase==RETURNING && inward && fabsf(x)<=exit_boundary){reset();return true;}
    return false;
  }
  float demand(float,const cart_motion::State &motion,
               float vmax,float amax,float jerk,float dt) const {
    const float target=phase==RETURNING ? -side*fminf(return_speed,vmax) : 0;
    const float cap=phase==RETURNING ? fminf(return_acceleration,amax) : amax;
    return cart_motion::velocityAccel(motion.velocity,target,cap,jerk,dt);
  }
};
}
