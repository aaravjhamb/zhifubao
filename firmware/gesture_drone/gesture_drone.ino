// gesture_drone.ino — DIY quad flight controller
//   * flies from the two BLE joycons (via the Mac bridge: python3 drone_bridge.py diy)
//   * serves a live web dashboard (attitude horizon, motors, arm/e-stop, PID tuning)
//
// Board: XIAO ESP32-S3 + 2x DRV8833 + MPU6050.
//
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!  PROPS OFF or drone tethered for ALL testing. Keep e-stop ready.  !!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// Wiring:
//   Motors  M1..M4 -> D0..D3 (GPIO1-4) -> DRV8833 IN1/IN3 ; IN2/IN4 -> GND
//   MPU6050 SDA->D4(GPIO5)  SCL->D5(GPIO6)  VCC->3V3  GND->GND
//   DRV8833 VM -> LiPo+ ; EEP -> 3V3 (always on) ; all grounds common
//
// USE:
//   1. Power the drone. It makes WiFi AP "gesture-drone" (pw flydrone).
//   2. Phone/laptop -> join that AP -> open http://192.168.4.1 for the dashboard.
//   3. Mac (also on that AP) -> python3 drone_bridge.py diy  to fly by gesture.
//   Arm/e-stop work from BOTH the web page and the bridge; e-stop always wins.

#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
// extra sensors (set to 0 to disable if one misbehaves right before a demo)
#define USE_TOF   1        // VL53L1X  (I2C 0x29) -> altitude.  Needs "VL53L1X" by Pololu
#define USE_FLOW  1        // PMW3901  (SPI)      -> drift.     Needs "Bitcraze PMW3901"
#if USE_TOF
  #include <VL53L1X.h>
#endif
#if USE_FLOW
  #include <SPI.h>
  #include "Bitcraze_PMW3901.h"
#endif

// ---- pins ----
const int PIN_M[4] = {1, 2, 3, 4};   // D0..D3 -> DRV8833 IN1/IN3
const int PIN_SDA  = 5;              // D4
const int PIN_SCL  = 6;              // D5
const int PIN_EEP  = -1;             // -1 = EEP hardwired to 3V3; set 44 for GPIO kill
const uint8_t MPU  = 0x68;
// PMW3901 optical-flow SPI pins (XIAO S3): SCK=7 MISO=8 MOSI=9, CS=43 (D6)
const int PIN_FLOW_CS = 43;

// ---- motor corners: index into PIN_M[] (PIN_M={1,2,3,4} = GPIO1..4) ----
// FL=GPIO3(idx2)  FR=GPIO4(idx3)  RL=GPIO2(idx1)  RR=GPIO1(idx0)
// Props reverse-mounted (diagonals same spin, adjacents opposite) -> no dir code needed.
const int MOT_FL = 2, MOT_FR = 3, MOT_RL = 1, MOT_RR = 0;

// ---- PWM ----
const int PWM_FREQ = 20000, PWM_RES = 10;    // duty 0..1023
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmInit(){ for(int i=0;i<4;i++){ ledcAttach(PIN_M[i],PWM_FREQ,PWM_RES); ledcWrite(PIN_M[i],0);} }
void pwmWrite(int i,int d){ ledcWrite(PIN_M[i],d); }
#else
void pwmInit(){ for(int i=0;i<4;i++){ ledcSetup(i,PWM_FREQ,PWM_RES); ledcAttachPin(PIN_M[i],i); ledcWrite(i,0);} }
void pwmWrite(int i,int d){ ledcWrite(i,d); }
#endif

// ---- tuning (live-editable from the web page) ----
float KP = 3.5f, KD = 0.8f, KYAW = 1.2f;
int         THR_MAX  = 900;          // flight throttle ceiling (of 1023). was 700
const int   THR_SPIN = 60;
const float MAX_ANGLE = 22.0f;
const float TILT_CUT  = 60.0f;

// ---- state ----
WiFiUDP  udp;
WebServer web(80);
bool  imuOK=false, armed=false, webEstop=false, testMode=false;
int   testVal[4]={0,0,0,0};          // per-motor test PWM (0..1023) when testMode
float ax,ay,az,gx,gy,gz, gbx=0,gby=0,gbz=0;
float pitch=0, roll=0;
int   mOut[4]={0,0,0,0};
float spRoll=0, spPitch=0, cmdYawRate=0; int cmdThr=0;
unsigned long lastPkt=0;

// extra sensors
#if USE_TOF
VL53L1X tof;
#endif
#if USE_FLOW
Bitcraze_PMW3901 flow(PIN_FLOW_CS);
#endif
bool  tofOK=false, flowOK=false;
int   altMM=0;                 // ToF altitude (mm), -1 = out of range
long  flowX=0, flowY=0;        // accumulated optical-flow drift
int16_t flowDX=0, flowDY=0;    // latest flow delta

void initSensors(){
#if USE_TOF
  tof.setBus(&Wire); tof.setTimeout(200);
  tofOK = tof.init();
  if(tofOK){ tof.setDistanceMode(VL53L1X::Long); tof.setMeasurementTimingBudget(33000); tof.startContinuous(33); }
  Serial.printf("ToF %s\n", tofOK?"OK":"absent");
#endif
#if USE_FLOW
  flowOK = flow.begin();       // uses default VSPI pins + PIN_FLOW_CS
  Serial.printf("Flow %s\n", flowOK?"OK":"absent");
#endif
}
void readSensors(){            // call ~20 Hz, NOT in the fast control loop
#if USE_TOF
  if(tofOK && tof.dataReady()){ int mm=tof.read(false); altMM = tof.timeoutOccurred()?altMM:mm; }
#endif
#if USE_FLOW
  if(flowOK){ int16_t dx,dy; flow.readMotionCount(&dx,&dy); flowDX=dx; flowDY=dy; flowX+=dx; flowY+=dy; }
#endif
}

// ---- IMU ----
void mpuW(uint8_t r,uint8_t v){ Wire.beginTransmission(MPU); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
bool mpuR(uint8_t r,uint8_t*b,uint8_t n){ Wire.beginTransmission(MPU); Wire.write(r);
  if(Wire.endTransmission(false)) return false; if(Wire.requestFrom(MPU,n)!=n) return false;
  for(uint8_t i=0;i<n;i++) b[i]=Wire.read(); return true; }
bool readIMU(){ uint8_t b[14]; if(!mpuR(0x3B,b,14)) return false;
  ax=(int16_t)(b[0]<<8|b[1])/16384.0f; ay=(int16_t)(b[2]<<8|b[3])/16384.0f; az=(int16_t)(b[4]<<8|b[5])/16384.0f;
  gx=(int16_t)(b[8]<<8|b[9])/131.0f-gbx; gy=(int16_t)(b[10]<<8|b[11])/131.0f-gby; gz=(int16_t)(b[12]<<8|b[13])/131.0f-gbz;
  return true; }
void calibrateIMU(){ Serial.println("IMU cal - keep still 2s");
  double sx=0,sy=0,sz=0,sax=0,say=0,saz=0; int n=0; gbx=gby=gbz=0;
  for(int i=0;i<400;i++){ if(readIMU()){sx+=gx;sy+=gy;sz+=gz;sax+=ax;say+=ay;saz+=az;n++;} delay(4);}
  if(n>100){ gbx=sx/n;gby=sy/n;gbz=sz/n; float mx=sax/n,my=say/n,mz=saz/n;
    pitch=atan2f(-mx,sqrtf(my*my+mz*mz))*57.2958f; roll=atan2f(my,mz)*57.2958f; } }
void attitude(float dt){ if(!readIMU()) return;
  pitch+=gy*dt; roll+=gx*dt;
  float m=sqrtf(ax*ax+ay*ay+az*az);
  if(m>0.8f&&m<1.2f){ float ap=atan2f(-ax,sqrtf(ay*ay+az*az))*57.2958f, ar=atan2f(ay,az)*57.2958f;
    pitch=0.98f*pitch+0.02f*ap; roll=0.98f*roll+0.02f*ar; } }

// ---- motors / arming ----
void eep(bool on){ if(PIN_EEP>=0) digitalWrite(PIN_EEP, on?HIGH:LOW); }
void motorsOff(){ for(int i=0;i<4;i++){ mOut[i]=0; pwmWrite(i,0);} }
void disarm(const char*w){ motorsOff(); eep(false); if(armed)Serial.printf("DISARM: %s\n",w); armed=false; }
void arm(){ if(webEstop){ Serial.println("arm refused: estop latched"); return; }
  if(cmdThr>THR_SPIN){ Serial.println("arm refused: throttle up"); return; }
  eep(true); armed=true; Serial.println("ARMED"); }

// ---- UDP command from the bridge ----
void parsePacket(){
  uint8_t b[16]; int n=udp.parsePacket(); if(n<8) return; udp.read(b,sizeof(b));
  if(b[0]!=0xAA||b[7]!=0x55) return;
  if((b[1]^b[2]^b[3]^b[4]^b[5])!=b[6]) return;
  spRoll  = (b[1]-128)/128.0f*MAX_ANGLE;
  spPitch = (b[2]-128)/128.0f*MAX_ANGLE;
  cmdThr  = b[3]<=128 ? 0 : map(b[3],128,255,0,THR_MAX);
  cmdYawRate = (b[4]-128)/128.0f*120.0f;
  bool wantArm=b[5]&1, estop=b[5]&2;
  lastPkt=millis();
  if(estop){ webEstop=true; disarm("bridge estop"); return; }
  if(wantArm && !armed) arm();
  if(!wantArm && armed) disarm("bridge disarm");
}

// ---- control loop ----
void control(float dt){
  if(!imuOK){ if(armed) disarm("no IMU"); return; }   // flight needs the IMU
  attitude(dt);
  if(armed && (fabsf(pitch)>TILT_CUT||fabsf(roll)>TILT_CUT)){ disarm("tilt"); return; }
  if(armed && millis()-lastPkt>500){ disarm("link lost"); return; }
  if(!armed || webEstop){ return; }
  if(cmdThr<THR_SPIN){ motorsOff(); return; }
  float eP=spPitch-pitch, eR=spRoll-roll;
  float pOut=KP*eP - KD*gy;
  float rOut=KP*eR - KD*gx;
  float yOut=KYAW*(cmdYawRate-gz);
  int m[4];
  m[MOT_FL]=cmdThr+pOut+rOut+yOut;
  m[MOT_FR]=cmdThr+pOut-rOut-yOut;
  m[MOT_RL]=cmdThr-pOut+rOut-yOut;
  m[MOT_RR]=cmdThr-pOut-rOut+yOut;
  for(int i=0;i<4;i++){ mOut[i]=constrain(m[i],THR_SPIN,1023); pwmWrite(i,mOut[i]); }
}

// ---- web dashboard ----
const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta name=viewport content="width=device-width,initial-scale=1">
<title>gesture drone</title><style>
body{font-family:system-ui,monospace;background:#0e0e10;color:#eee;margin:0;padding:14px;text-align:center}
h1{font-size:17px;margin:0 0 8px}
#stat{font-weight:700;padding:4px 10px;border-radius:6px}
.armed{background:#7a1f1f;color:#fdd}.safe{background:#245c2e;color:#cfc}.estop{background:#b00;color:#fff}
.wrap{display:flex;gap:14px;justify-content:center;flex-wrap:wrap;align-items:flex-start}
canvas{background:#161618;border-radius:10px}
.quad{position:relative;width:170px;height:170px;background:#161618;border-radius:10px;margin:auto}
.m{position:absolute;width:44px;height:44px;border-radius:50%;background:#333;display:flex;align-items:center;justify-content:center;font-size:12px;color:#bb8}
.arm-btns button,.est button{font-family:inherit;border:0;border-radius:8px;padding:14px;font-size:16px;margin:4px;cursor:pointer;width:130px}
#bArm{background:#245c2e;color:#cfc}#bDis{background:#333;color:#eee}
#bEst{background:#b00;color:#fff;font-weight:700;width:270px;font-size:18px}
.gains{margin-top:8px;font-size:13px}.gains label{display:inline-block;width:270px;margin:3px}
.gains input{vertical-align:middle;width:150px}
.mrow{margin:5px;font-size:14px}.mrow b{color:#8af;display:inline-block;width:26px}
.mrow input[type=range]{vertical-align:middle;width:130px}
.mrow input[type=number]{width:60px;background:#222;color:#eee;border:1px solid #444;border-radius:4px;margin:0 4px}
.mrow .mv{display:inline-block;width:44px}
small{color:#888}
</style></head><body>
<h1>gesture drone <span id=stat class=safe>--</span></h1>
<div class=wrap>
  <canvas id=horizon width=200 height=200></canvas>
  <div>
    <div class=quad id=quad>
      <div class=m id=mFL style="top:8px;left:8px">FL</div>
      <div class=m id=mFR style="top:8px;right:8px">FR</div>
      <div class=m id=mRL style="bottom:8px;left:8px">RL</div>
      <div class=m id=mRR style="bottom:8px;right:8px">RR</div>
    </div>
    <div id=vals style="margin-top:6px;font-size:13px"></div>
    <div id=sens style="margin-top:4px;font-size:13px;color:#8fb"></div>
  </div>
</div>
<div class=arm-btns>
  <button id=bArm onclick="cmd('arm=1')">ARM</button>
  <button id=bDis onclick="cmd('arm=0')">DISARM</button>
</div>
<div class=est><button id=bEst onclick="cmd('estop=1')">EMERGENCY STOP</button>
  <div><small>after e-stop, press DISARM then ARM to clear</small></div></div>

<div id=testpanel style="margin-top:10px;border-top:1px solid #333;padding-top:8px">
  <button id=bTest onclick="toggleTest()" style="font-family:inherit;background:#555;color:#fff;border:0;border-radius:8px;padding:12px;font-size:15px;width:200px;cursor:pointer">MOTOR TEST: off</button>
  <button onclick="allOff()" style="font-family:inherit;background:#333;color:#eee;border:0;border-radius:8px;padding:12px;font-size:14px;margin-left:4px;cursor:pointer">ALL OFF</button>
  <div id=sliders style="margin-top:8px;opacity:.4;pointer-events:none">
    <!-- idx: 2=FL(GPIO3) 3=FR(GPIO4) 1=RL(GPIO2) 0=RR(GPIO1) -->
    <div class=mrow><b>FL</b> GPIO3 <input type=range class=mt data-i=2 min=0 max=1023 value=0><input type=number class=mn data-i=2 min=0 max=1023 value=0><span class=mv>0%</span></div>
    <div class=mrow><b>FR</b> GPIO4 <input type=range class=mt data-i=3 min=0 max=1023 value=0><input type=number class=mn data-i=3 min=0 max=1023 value=0><span class=mv>0%</span></div>
    <div class=mrow><b>RL</b> GPIO2 <input type=range class=mt data-i=1 min=0 max=1023 value=0><input type=number class=mn data-i=1 min=0 max=1023 value=0><span class=mv>0%</span></div>
    <div class=mrow><b>RR</b> GPIO1 <input type=range class=mt data-i=0 min=0 max=1023 value=0><input type=number class=mn data-i=0 min=0 max=1023 value=0><span class=mv>0%</span></div>
    <div class=mrow><b>ALL</b> <input type=number id=mall min=0 max=1023 value=0><button onclick="setAll()" style="font-family:inherit;background:#2a2a2e;color:#8af;border:1px solid #444;border-radius:6px;padding:6px 12px">set all</button></div>
  </div>
  <small>raw PWM 0-1023 · test mode disarms flight · props OFF · e-stop kills it</small>
</div>
<div class=gains>
  <label>KP <input type=range id=kp min=0 max=8 step=0.1></label>
  <label>KD <input type=range id=kd min=0 max=3 step=0.05></label>
  <label>KYAW <input type=range id=ky min=0 max=4 step=0.1></label>
  <button onclick="applyGains()" style="font-family:inherit;background:#2a2a2e;color:#8af;border:1px solid #444;border-radius:6px;padding:8px 14px">apply gains</button>
</div>
<script>
const H=document.getElementById('horizon'),hx=H.getContext('2d');
let rP=0,rR=0;
function drawHorizon(){
  const w=200,h=200,cx=100,cy=100;
  hx.clearRect(0,0,w,h); hx.save();
  hx.translate(cx,cy); hx.rotate(-rR*Math.PI/180);
  const off=rP*2;
  hx.fillStyle='#3a6ea5'; hx.fillRect(-200,-200+off,400,200);   // sky
  hx.fillStyle='#6b4a2b'; hx.fillRect(-200,off,400,200);        // ground
  hx.strokeStyle='#fff'; hx.lineWidth=2; hx.beginPath(); hx.moveTo(-200,off); hx.lineTo(200,off); hx.stroke();
  hx.restore();
  hx.strokeStyle='#ff0'; hx.lineWidth=3; hx.beginPath();
  hx.moveTo(70,100); hx.lineTo(90,100); hx.moveTo(110,100); hx.lineTo(130,100);
  hx.moveTo(100,100); hx.lineTo(100,108); hx.stroke();
  requestAnimationFrame(drawHorizon);
}
requestAnimationFrame(drawHorizon);
function motorColor(v){ const t=Math.min(1,v/1023); const g=Math.round(60+t*160);
  return 'rgb('+Math.round(40+t*180)+','+g+',60)'; }
async function tick(){
 try{ const d=await(await fetch('/data')).json();
  rP=d.pitch; rR=d.roll;
  const s=document.getElementById('stat');
  if(d.estop){s.textContent='E-STOP';s.className='estop';}
  else if(d.armed){s.textContent=d.link?'ARMED':'ARMED (no link)';s.className='armed';}
  else{s.textContent=d.link?'SAFE · linked':'SAFE · no bridge';s.className='safe';}
  const ids=['mFL','mFR','mRL','mRR'];
  for(let i=0;i<4;i++){const e=document.getElementById(ids[i]);e.style.background=motorColor(d.m[i]);
    e.textContent=Math.round(d.m[i]/1023*100)+'%';}
  document.getElementById('vals').textContent=
    'roll '+d.roll.toFixed(0)+'°  pitch '+d.pitch.toFixed(0)+'°  thr '+Math.round(d.thr/7)+'%';
  let ss=[];
  if(d.tofOK)  ss.push('alt '+(d.alt>0?d.alt+' mm':'--'));
  if(d.flowOK) ss.push('flow '+d.fdx+','+d.fdy);
  document.getElementById('sens').textContent=ss.join('   ');
 }catch(e){}
}
setInterval(tick,100);
async function cmd(q){ await fetch('/cmd?'+q); }
async function applyGains(){ await fetch('/gains?kp='+kp.value+'&kd='+kd.value+'&ky='+ky.value); }
fetch('/data').then(r=>r.json()).then(d=>{kp.value=d.kp;kd.value=d.kd;ky.value=d.kyaw;});
let testOn=false;
async function toggleTest(){
  testOn=!testOn;
  await fetch('/test?on='+(testOn?1:0));
  document.getElementById('bTest').textContent='MOTOR TEST: '+(testOn?'ON':'off');
  document.getElementById('bTest').style.background=testOn?'#b06000':'#555';
  const s=document.getElementById('sliders');
  s.style.opacity=testOn?'1':'.4'; s.style.pointerEvents=testOn?'auto':'none';
  if(!testOn) allOff();
}
async function allOff(){
  document.querySelectorAll('.mt').forEach(sl=>{sl.value=0;});
  document.querySelectorAll('.mn').forEach(nb=>{nb.value=0;});
  document.querySelectorAll('.mv').forEach(v=>{v.textContent='0%';});
  for(let i=0;i<4;i++) await fetch('/motor?i='+i+'&v=0');
}
function applyMotor(i,v){
  v=Math.max(0,Math.min(1023,v|0));
  document.querySelector('.mt[data-i="'+i+'"]').value=v;
  document.querySelector('.mn[data-i="'+i+'"]').value=v;
  document.querySelector('.mt[data-i="'+i+'"]').parentElement.querySelector('.mv').textContent=Math.round(v/1023*100)+'%';
  fetch('/motor?i='+i+'&v='+v);
}
document.querySelectorAll('.mt').forEach(sl=>{ sl.addEventListener('input',()=>applyMotor(sl.dataset.i,+sl.value)); });
document.querySelectorAll('.mn').forEach(nb=>{ nb.addEventListener('input',()=>applyMotor(nb.dataset.i,+nb.value)); });
function setAll(){ const v=+document.getElementById('mall').value; for(let i=0;i<4;i++) applyMotor(i,v); }
</script></body></html>
)HTML";

void handleData(){
  bool link = (millis()-lastPkt < 500);
  char b[340];
  snprintf(b,sizeof(b),
    "{\"armed\":%d,\"estop\":%d,\"link\":%d,\"roll\":%.1f,\"pitch\":%.1f,\"thr\":%d,"
    "\"m\":[%d,%d,%d,%d],\"kp\":%.2f,\"kd\":%.2f,\"kyaw\":%.2f,"
    "\"tofOK\":%d,\"alt\":%d,\"flowOK\":%d,\"fdx\":%d,\"fdy\":%d,\"test\":%d}",
    armed, webEstop, link, roll, pitch, cmdThr, mOut[0],mOut[1],mOut[2],mOut[3], KP,KD,KYAW,
    tofOK, altMM, flowOK, flowDX, flowDY, testMode);
  web.send(200,"application/json",b);
}
void handleCmd(){
  if(web.hasArg("estop")){ webEstop=true; testMode=false; for(int i=0;i<4;i++)testVal[i]=0; disarm("web estop"); }
  if(web.hasArg("arm")){
    if(web.arg("arm").toInt()){ webEstop=false; arm(); }
    else disarm("web");
  }
  web.send(200,"text/plain","ok");
}
void handleGains(){
  if(web.hasArg("kp")) KP=web.arg("kp").toFloat();
  if(web.hasArg("kd")) KD=web.arg("kd").toFloat();
  if(web.hasArg("ky")) KYAW=web.arg("ky").toFloat();
  web.send(200,"text/plain","ok");
}
// motor bench test: /test?on=1|0   and   /motor?i=<0..3>&v=<0..1023>
void handleTest(){
  if(web.hasArg("on")){
    testMode = web.arg("on").toInt()!=0;
    disarm("test mode");                       // flight always off during test
    for(int i=0;i<4;i++){ testVal[i]=0; pwmWrite(i,0); }
    webEstop=false;
  }
  web.send(200,"text/plain", testMode?"test on":"test off");
}
void handleMotor(){
  if(testMode && web.hasArg("i") && web.hasArg("v")){
    int i=web.arg("i").toInt(); int v=constrain(web.arg("v").toInt(),0,1023);
    if(i>=0&&i<4) testVal[i]=v;
  }
  web.send(200,"text/plain","ok");
}

void setup(){
  if(PIN_EEP>=0){ pinMode(PIN_EEP,OUTPUT); digitalWrite(PIN_EEP,LOW); }
  pwmInit();
  Serial.begin(115200); delay(300);
  Serial.println("\n=== gesture_drone ===");
  Wire.begin(PIN_SDA,PIN_SCL,(uint32_t)400000);
  mpuW(0x6B,0x00); mpuW(0x1A,0x03);
  uint8_t who=0; imuOK=mpuR(0x75,&who,1);
  Serial.printf("IMU %s (0x%02X)\n", imuOK?"OK":"FAIL", who);
  if(imuOK) calibrateIMU();
  initSensors();
  WiFi.softAP("gesture-drone-test","flydrone");
  Serial.print("AP up: gesture-drone  dashboard http://"); Serial.println(WiFi.softAPIP());
  udp.begin(9000);
  web.on("/", [](){ web.send_P(200,"text/html",PAGE); });
  web.on("/data", handleData);
  web.on("/cmd", handleCmd);
  web.on("/gains", handleGains);
  web.on("/test", handleTest);
  web.on("/motor", handleMotor);
  web.begin();
  Serial.println("bridge: python3 drone_bridge.py diy");
}

void loop(){
  web.handleClient();
  parsePacket();
  if(testMode){                       // bench test: drive pins directly, no control-loop/IMU dependency
    for(int i=0;i<4;i++){ int v=webEstop?0:testVal[i]; mOut[i]=v; pwmWrite(i,v); }
  } else {
    static unsigned long lu=0; unsigned long now=micros();
    if(now-lu>=4000){ float dt=(now-lu)/1e6f; lu=now; if(dt<0.05f) control(dt); }
  }
  static unsigned long ls=0;
  if(millis()-ls>=50){ ls=millis(); readSensors(); }   // 20 Hz sensor telemetry
  static unsigned long dbg=0;
  if(millis()-dbg>=300){ dbg=millis();
    Serial.printf("%s%s thr%4d sp[%+.0f %+.0f] att[%+.0f %+.0f] m %d %d %d %d\n",
      webEstop?"ESTOP ":"", armed?"ARM":"safe", cmdThr, spRoll, spPitch, roll, pitch,
      mOut[0],mOut[1],mOut[2],mOut[3]); }
}
