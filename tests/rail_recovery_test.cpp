#include "../swingup/rail_recovery.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
void scenario(float initial_x,float initial_v,float initial_a,float drive,float jerk){
 cart_motion::State m;m.velocity=initial_v;m.acceleration=initial_a;
 rail_recovery::State recovery;
 float x=initial_x;bool started=false,centered=false,done=false;float peak=fabsf(x);
 for(int i=0;i<10000;++i){
  if(recovery.phase==rail_recovery::NONE && recovery.atEdge(x,.15f)){recovery.begin();started=true;}
  float request=x<0?-drive:drive;
  if(recovery.phase!=rail_recovery::NONE){
   if(recovery.update(x,m,.001f)){done=true;assert(centered);assert(fabsf(x)<=.003f);assert(fabsf(m.velocity)<=.01f);break;}
   centered|=recovery.phase==rail_recovery::CENTERING;
   assert(!recovery.timedOut());
   request=recovery.demand(x,m,.75f,drive,jerk,.001f);
  }
  const auto previous=m;
  m=cart_motion::advance(m,x,request,.75f,drive,drive,jerk,.15f,.001f);
  if(m.brake_direction && recovery.phase==rail_recovery::NONE){recovery.begin();started=true;}
  x+=m.velocity*.001f;peak=fmaxf(peak,fabsf(x));
  assert(fabsf(m.acceleration-previous.acceleration)<=jerk*.001f+1e-5f);
  assert(fabsf(m.velocity)<=.752f);assert(fabsf(x)<.14f);
 }
 assert(started&&done);
 printf("x0=%+.3f v0=%+.3f: stop/recenter complete; peak |x|=%.3f mm\n",initial_x,initial_v,peak*1000);
}
int main(){
 rail_recovery::State r;
 assert(!r.atEdge(.129f,.15f));assert(r.atEdge(.131f,.15f));
 // Low-speed motion may use the outer part of the central 260 mm normally.
 cart_motion::State low;low.velocity=.01f;
 assert(cart_motion::advance(low,.129f,0,.75f,15,15,50,.15f,.001f).brake_direction==0);
 for(int sign:{-1,1}){
  scenario(sign*.01f,sign*.75f,0,15,50);
  scenario(sign*.129f,sign*.1f,0,15,50);
  scenario(sign*.131f,0,0,15,50);
  scenario(sign*.131f,-sign*.4f,0,15,50);
  scenario(sign*.132f,0,0,.5f,10);
 }
 r.begin();for(int i=0;i<8100;++i)r.update(.10f,{},.001f);
 assert(r.timedOut());r.reset();assert(r.phase==rail_recovery::NONE&&!r.timedOut());
 assert(fabsf(cart_motion::stoppingDistance(.75f,0,15,50)-.08660254f)<1e-6f);
}
