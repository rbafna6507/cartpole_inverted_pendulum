"""Compile the real driver functions with mocked GPIO and verify stop behavior."""
from pathlib import Path
import subprocess,tempfile,unittest,re
ROOT=Path(__file__).resolve().parents[1]
SOURCE=(ROOT/'swingup/swingup.ino').read_text()
def function(name):
    match=re.search(r'^void (?:IRAM_ATTR )?'+name+r'\(',SOURCE,re.M); start=match.start();pos=match.end()
    brace=SOURCE.index('{',pos); depth=1; end=brace+1
    while depth:
        depth+=(SOURCE[end]=='{')-(SOURCE[end]=='}');end+=1
    return SOURCE[start:end]
class DriverDisableTest(unittest.TestCase):
    def test_stop_suppresses_enable_pulses_and_position_counts(self):
        setup=function('setup'); prefix=setup[setup.index('{')+1:setup.index('Serial.begin')]
        self.assertNotIn('energize(true)',setup)
        self.assertNotIn('profileFast',SOURCE)
        self.assertNotIn('fast_profile',SOURCE)
        self.assertIn('!strcmp(cmd,"stop"))  { eStop();',SOURCE)
        harness=r'''
#include <cstdint>
#include <cmath>
#include <cassert>
#define IRAM_ATTR
#define HIGH 1
#define LOW 0
#define GPIO_MODE_OUTPUT 1
#define OUTPUT 1
#define GPIO_OUT_W1TC_REG 0
#define GPIO_OUT_W1TS_REG 1
using gpio_num_t=int;
constexpr int PIN_EN_ALL=27,N_AX=3,ISR_HZ=125000;
const uint32_t AX_MASK[]={1u<<18,1u<<25,1u<<16};
const uint32_t ALL_STEP_MASK=AX_MASK[0]|AX_MASK[1]|AX_MASK[2];
const uint8_t DIR_PIN[]={19,26,17};
volatile bool g_energized=false;
volatile uint32_t ax_inc[3]={}; uint32_t ax_acc[3]={};
volatile int32_t ax_pos[3]={}; volatile int8_t ax_dir[3]={1,1,1};
int enable_level=0,pulse_writes=0;
bool arduino_gpio_registered=false;
void digitalWrite(int pin,int level){if(pin==PIN_EN_ALL && arduino_gpio_registered)enable_level=level;}
void gpio_set_level(int pin,int level){if(pin==PIN_EN_ALL)enable_level=level;}
void gpio_set_direction(int,int){assert(enable_level==HIGH);}
void pinMode(int pin,int){assert(enable_level==HIGH);if(pin==PIN_EN_ALL)arduino_gpio_registered=true;}
void reg_write(int reg,uint32_t){if(reg==GPIO_OUT_W1TS_REG)++pulse_writes;}
#define REG_WRITE(r,v) reg_write(r,v)
enum {M_IDLE,M_SWINGUP}; int mode=M_SWINGUP,cart_brake_direction=1;
struct Pump {void reset(){}} pump,rail_recovery_state;
bool recovery_was_manual=false;
struct Response {bool armed=true;} response_watch;
float v_manual=1,vc=1,acc_cmd=1,zv_target=1,zv_now=1;
'''
        harness+='\n'+function('energize')+'\n'+function('axSetRate')+'\n'+function('onStepTimer')+'\n'+function('eStop')
        harness+='\nint main(){\n'+prefix+r'''
 assert(!g_energized && enable_level==HIGH && arduino_gpio_registered);
 energize(true);assert(enable_level==LOW);
 for(int i=0;i<3;++i){axSetRate(i,30000,false);assert(ax_inc[i]);}
 eStop();assert(!g_energized && enable_level==HIGH && mode==M_IDLE);
 assert(vc==0 && acc_cmd==0 && zv_target==0 && zv_now==0);
 for(int i=0;i<3;++i){assert(ax_inc[i]==0);axSetRate(i,30000,false);assert(ax_inc[i]==0);}
 // Simulate a stale in-flight rate write after stop; the ISR must still gate it.
 for(int i=0;i<3;++i){ax_inc[i]=0x80000000u;ax_acc[i]=0xffffffffu;}
 for(int n=0;n<20;++n)onStepTimer();
 assert(pulse_writes==0);for(int i=0;i<3;++i)assert(ax_pos[i]==0);
}
'''
        with tempfile.TemporaryDirectory() as d:
            cpp=Path(d)/'test.cpp';exe=Path(d)/'test';cpp.write_text(harness)
            subprocess.run(['clang++','-std=c++11','-Wall','-Wextra','-Werror',str(cpp),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
if __name__=='__main__':unittest.main()
