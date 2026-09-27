#include "Arduino.h"
unsigned long g_us = 1000;
MockSerial Serial; 
#include "Wire.h"
MockWire Wire;
double g_roll_deg=0, g_pitchchip_deg=0, g_gyro_dps[3]={0,0,0}, g_bias[3]={0,0,0}, g_noise=0;
#include "../veg_flight_controller/veg_flight_controller.ino"
#include <cassert>
int fails=0;
#define CHECK(c,msg) do{ bool _ok=(c); printf("%s  %s\n", _ok?"PASS":"FAIL", msg); if(!_ok) fails++; }while(0)
void run_ms(int ms){ int ticks=ms*1000/LOOP_US; for(int i=0;i<ticks;i++){ g_us+=LOOP_US; loop(); } }
void send(const char*s){ Serial.feed(s); }
void hb_run(int ms,const char*set){ for(int t=0;t<ms;t+=100){ send(set); run_ms(100);} }
bool outHas(const char*s){ return Serial.out.find(s)!=std::string::npos; }
int main(){
  g_bias[0]=300; g_bias[1]=-200; g_bias[2]=150; g_noise=0.3;
  setup();
  CHECK(esc[0].firstUs==1000 && esc[3].firstUs==1000, "ESCs see 1000us from the first pulse (not Servo's 1500 default)");
  CHECK(imuOk, "gyro calibration succeeds when still");
  run_ms(5000);
  CHECK(fabs(roll)<0.5 && fabs(pitch)<0.5, "no attitude drift after 5 s despite gyro bias (was ~4.6 deg/s drift)");
  g_roll_deg=10; run_ms(3000);
  CHECK(fabs(roll-10)<0.5, "accel fusion tracks a static 10 deg roll");
  g_roll_deg=0; g_pitchchip_deg=-8; run_ms(3000);
  CHECK(fabs(pitch-8)<0.5, "nose-up tilt reads as +pitch");
  g_pitchchip_deg=0; run_ms(3000);

  Serial.out.clear(); send("ARM\n"); run_ms(8);
  CHECK(state==DISARMED && outHas("ERR send SET first"), "ARM refused before any SET heartbeat");
  send("SET 1.5 -2.25 0 0\n"); run_ms(8);
  CHECK(fabs(rollSet-1.5)<1e-4 && fabs(pitchSet+2.25)<1e-4, "SET floats parse correctly (sscanf %f did not on AVR)");
  Serial.out.clear(); send("SET 5 -3 0 1.2\n"); run_ms(8);
  CHECK(outHas("ERR throttle must be 0..1") && throttleSet==0, "old 'SET r p y 1.2' (altitude) rejected, not full throttle");
  Serial.out.clear(); send("SET 1 2\n"); send("SET a b c d\n"); run_ms(8);
  CHECK(outHas("ERR SET needs 4 values") && outHas("ERR SET bad number"), "malformed SET lines rejected");
  send("SET 0 0 0 0.5\n"); run_ms(8); Serial.out.clear(); send("ARM\n"); run_ms(8);
  CHECK(state==DISARMED && outHas("ERR throttle not low"), "ARM refused with throttle high");
  g_roll_deg=20; run_ms(3000); send("SET 0 0 0 0\n"); run_ms(8); Serial.out.clear(); send("ARM\n"); run_ms(8);
  CHECK(state==DISARMED && outHas("ERR not level"), "ARM refused when tilted");
  g_roll_deg=0; run_ms(3000);
  CHECK(motorUs[0]==1000 && motorUs[2]==1000, "disarmed -> all motors 1000us");
  send("SET 0 0 0 0\n"); run_ms(8); send("ARM\n"); run_ms(8);
  CHECK(state==ARMED, "ARM accepted: level, throttle low, link alive");
  hb_run(300,"SET 0 0 0 0\n");
  CHECK(motorUs[0]>=1100 && motorUs[0]<1110, "armed at zero throttle -> idle ~1100us");

  hb_run(300,"SET 10 0 0 0.5\n");
  CHECK(motorUs[0]>motorUs[1] && motorUs[3]>motorUs[2], "roll-right command speeds up LEFT motors (M1,M4)");
  hb_run(300,"SET 0 10 0 0.5\n");
  CHECK(motorUs[2]>motorUs[1] && motorUs[3]>motorUs[0], "nose-up command speeds up FRONT motors (M3,M4)");
  hb_run(300,"SET 0 0 30 0.5\n");
  CHECK(motorUs[0]>motorUs[1] && motorUs[2]>motorUs[3], "yaw-right command speeds up CCW-prop motors (M1,M3)");

  // Link loss in the air
  hb_run(500,"SET 0 0 0 0.6\n"); Serial.out.clear();
  run_ms(600);
  CHECK(state==FAILSAFE && outHas("FDI FAILSAFE link lost"), "link lost 500 ms while flying -> FAILSAFE");
  float prev=fsThrottle; bool neverUp=true;
  for(int i=0;i<30;i++){ run_ms(100); if(fsThrottle>prev+1e-6) neverUp=false; prev=fsThrottle; }
  CHECK(neverUp && fabs(fsThrottle-FS_THROTTLE)<0.01, "failsafe throttle only ramps DOWN to FS_THROTTLE");
  run_ms(FS_DESCENT_MS);
  CHECK(state==DISARMED && motorUs[0]==1000, "failsafe ends with motors stopped");

  // Link loss on the ground
  send("SET 0 0 0 0\n"); run_ms(8); send("ARM\n"); run_ms(8); run_ms(700);
  CHECK(state==DISARMED, "link lost while armed on the ground -> immediate disarm (no climb)");

  // Integrator windup (PID mode)
  send("MODE PID\n"); send("SET 0 0 0 0\n"); run_ms(8); send("ARM\n"); run_ms(8);
  CHECK(state==ARMED && controlMode==CONTROL_MODE_PID, "MODE PID accepted while disarmed");
  g_roll_deg=-8; for(int t=0;t<20000;t+=100){ send("SET 5 0 0 0.5\n"); run_ms(100);} 
  CHECK(fabs(iRoll)<=I_LIMIT_US+1e-3, "roll integrator clamped (anti-windup)");
  Serial.out.clear(); hb_run(1500,"SET 5 -25 0 0.5\n");
  // measured pitch 0 vs set -25 isn't >25, use roll error instead
  g_roll_deg=-25; hb_run(4000,"SET 5 0 0 0.5\n");
  CHECK(state==FAILSAFE && outHas("attitude tracking lost"), "attitude can't follow for 0.5 s -> FAILSAFE (rotor-loss FDI)");
  send("DISARM\n"); run_ms(8);
  CHECK(state==DISARMED, "DISARM works in FAILSAFE");

  // crash tilt
  g_roll_deg=0; run_ms(4000); send("SET 0 0 0 0\n"); run_ms(8); send("ARM\n"); run_ms(8);
  g_roll_deg=75; for(int t=0;t<1500;t+=100){ send("SET 0 0 0 0.5\n"); run_ms(100);} 
  CHECK(state==DISARMED && motorUs[1]==1000, "flip >60 deg -> motors cut");

  // I2C failure
  g_roll_deg=0; run_ms(4000); send("SET 0 0 0 0\n"); run_ms(8); send("ARM\n"); run_ms(8);
  Wire.timeoutFlag=true; hb_run(100,"SET 0 0 0 0.4\n"); Wire.timeoutFlag=false;
  CHECK(state==DISARMED, "I2C/IMU read failure -> disarm");

  // long garbage line doesn't break parser
  std::string junk(200,'x'); send((junk+"\n").c_str()); send("SET 0 0 0 0\n"); run_ms(8);
  CHECK(millis()-lastSetMs<20, "overlong line dropped, next command still parsed");

  printf("\n%d failure(s)\n", fails); return fails;
}
