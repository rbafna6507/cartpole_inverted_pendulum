"""Compile the actual encTheta implementation and verify the measured target."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
SOURCE=(ROOT/'swingup/swingup.ino').read_text()
class EncoderReferenceTest(unittest.TestCase):
    def test_measured_upright_and_zero_independence(self):
        start=SOURCE.index('float encTheta() {');end=SOURCE.index('\n}',start)+2
        harness='''#include <cassert>
#include <cmath>
#include "'''+str(ROOT/'swingup/encoder_reference.h')+'''"
#define ENC_INVERT 0
int32_t enc_last_raw=0,enc_ticks=0,enc_zero=0;
'''+SOURCE[start:end]+'''
int main(){
 const float scale=encoder_reference::radians_per_count;
 // The actual entry point used by estimate(), capture and balance.
 enc_last_raw=3416;assert(encTheta()==0);
 for(int z: {0,1403,4095,-8192}){
   enc_zero=z;enc_ticks=z+2000;assert(encTheta()==0);
 }
 enc_last_raw=3417;assert(std::abs(encTheta()-scale)<1e-6);
 enc_last_raw=3415;assert(std::abs(encTheta()+scale)<1e-6);
 enc_last_raw=0;float a=encTheta();enc_last_raw=4095;float b=encTheta();
 assert(std::abs(a-b-scale)<1e-6);
 enc_last_raw=597;assert(encTheta()>0); // The measured +cart horizontal pose.
 enc_last_raw=2650;assert(encTheta()<0);
 enc_last_raw=3416;assert(encoder_reference::angle_from_upright(enc_last_raw,true)==0);
 for(int raw=0;raw<4096;raw++){
   float normal=encoder_reference::angle_from_upright(raw,false);
   assert(std::abs(normal)<=3.141593f);
   assert(encoder_reference::angle_from_upright(raw,true)==-normal);
 }
}
'''
        harness='#include <initializer_list>\n'+harness
        with tempfile.TemporaryDirectory() as d:
            cpp=Path(d)/'reference.cpp';exe=Path(d)/'reference';cpp.write_text(harness)
            subprocess.run(['clang++','-std=c++11','-Wall','-Wextra','-Werror',str(cpp),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
if __name__=='__main__':unittest.main()
