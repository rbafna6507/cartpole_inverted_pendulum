// Host-side reference fixtures for the JavaScript firmware-equation port.
#include "../swingup/swing_controller.h"
#include <cstdio>
#include <cmath>
int main(){
  swing_control::Parameters p;auto k=swing_control::gains(p);
  std::printf("geometry %d %.9g %.9g %.9g %.9g\n",cart_hardware::pulley_teeth,cart_hardware::steps_per_m,p.vmax,p.amax_s,p.jmax);
  std::printf("gain %.9g %.9g %.9g %.9g\n",k.k1,k.k2,k.k3,k.k4);
  auto uk=swing_control::uprightGains(p);
  std::printf("upright_gain %.9g %.9g %.9g %.9g\n",uk.k1,uk.k2,uk.k3,uk.k4);
  for(int i=0;i<100;++i){
    float x=.13f*std::sin(i*.37f),request=8*std::sin(i*.61f);
    cart_motion::State s;s.velocity=1.2f*std::sin(i*.19f);s.acceleration=6*std::cos(i*.23f);
    auto n=cart_motion::advance(s,x,request,1.5f,6,6,150,.15f,.001f);
    std::printf("motion %.9g %.9g %.9g %.9g %.9g %.9g %d\n",x,request,s.velocity,s.acceleration,n.velocity,n.acceleration,n.brake_direction);
  }
  for(int i=0;i<100;++i){
    float theta=.65f*std::sin(i*.71f),omega=3.2f*std::cos(i*.43f);
    float x=.13f*std::sin(i*.37f),v=.75f*std::sin(i*.19f),a=15*std::cos(i*.23f);
    std::printf("capture %.9g %.9g %.9g %.9g %.9g %d\n",theta,omega,x,v,a,
      swing_control::canCapture(p,k,theta,omega,x,v,a));
  }
}
