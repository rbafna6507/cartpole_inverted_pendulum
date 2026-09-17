#include "../swingup/rail_recovery.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
void scenario(float initial_x,float initial_v,float initial_a,float drive,float jerk,float vmax=.8f){
 cart_motion::State m;m.velocity=initial_v;m.acceleration=initial_a;
 rail_recovery::State recovery;
 float x=initial_x;bool started=false,done=false;float peak=fabsf(x);
 for(int i=0;i<10000;++i){
  if(recovery.phase==rail_recovery::NONE && recovery.atEdge(x,.15f)){recovery.begin(x,.15f);started=true;}
  float request=x<0?-drive:drive;
  if(recovery.phase!=rail_recovery::NONE){
   if(recovery.update(x,m,.001f)){done=true;assert(fabsf(x)<=.13001f);assert(m.velocity*initial_x<=.0007f);break;}
   assert(!recovery.timedOut());
   request=recovery.demand(x,m,vmax,drive,jerk,.001f);
  }
  const auto previous=m;
  m=cart_motion::advance(m,x,request,vmax,drive,drive,jerk,.15f,.001f);
  if(m.brake_direction && recovery.phase==rail_recovery::NONE){recovery.begin(x,.15f,m.brake_direction);started=true;}
  x+=m.velocity*.001f;peak=fmaxf(peak,fabsf(x));
  assert(fabsf(m.acceleration-previous.acceleration)<=jerk*.001f+1e-5f);
  assert(fabsf(m.velocity)<=vmax+.002f);assert(fabsf(x)<.14f);
 }
 assert(started&&done);
 // Edge-started recovery must finish near its boundary without seeking center.
 if(fabsf(initial_x)>.135f)assert(fabsf(x)>.12f);
 printf("x0=%+.3f v0=%+.3f: return complete at %+.3f mm; peak |x|=%.3f mm\n",initial_x,initial_v,x*1000,peak*1000);
}
int main(){
 rail_recovery::State r;
 assert(!r.atEdge(.134f,.15f));assert(r.atEdge(.136f,.15f));
 cart_motion::State low;low.velocity=.01f;
 assert(cart_motion::advance(low,.134f,0,.75f,15,15,50,.15f,.001f).brake_direction==0);
 for(int sign:{-1,1}){
  scenario(sign*.01f,sign*.8f,0,18,100);
  scenario(sign*.130f,sign*.1f,0,18,100);
  scenario(sign*.136f,0,0,18,100);
  scenario(sign*.136f,-sign*.4f,0,18,100);
  scenario(sign*.137f,0,0,.5f,10);
 }
 r.begin(.137f,.15f);for(int i=0;i<8100;++i)r.update(.137f,{},.001f);
 assert(r.timedOut());r.reset();assert(r.phase==rail_recovery::NONE&&!r.timedOut());
 // A fast approach that reverses early can resume without going to either boundary.
 r.begin(.05f,.15f,1);cart_motion::State inward;inward.velocity=-.01f;inward.acceleration=-1;
 assert(r.update(.06f,inward,.001f));
 // Crossing the boundary while still accelerating outward must not release.
 r.begin(.136f,.15f,1);cart_motion::State outward;outward.velocity=.004f;outward.acceleration=1;
 assert(!r.update(.129f,outward,.001f));
 assert(fabsf(cart_motion::stoppingDistance(.75f,0,15,50)-.08660254f)<1e-6f);
}
