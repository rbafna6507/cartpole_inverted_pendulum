#include "../swingup/swing_controller.h"
#include <cassert>
int main(){
 swing_control::Parameters p; const auto k=swing_control::gains(p);
 assert(swing_control::canCapture(p,k,.55f,-1.0f,0,0,0)); // 31.5 deg: earlier inbound catch
 assert(!swing_control::canCapture(p,k,.55f,1.0f,0,0,0)); // departing enlarged window
 assert(!swing_control::canCapture(p,k,.05f,-1,0,.79f,0)); // reserve velocity headroom
 assert(!swing_control::canCapture(p,k,.1f,0,.13f,0,0)); // rail room
 assert(!swing_control::canCapture(p,k,.59f,-.1f,0,0,-15)); // can't reverse acceleration in time
 assert(swing_control::canCapture(p,k,0,0,0,0,0));
}
