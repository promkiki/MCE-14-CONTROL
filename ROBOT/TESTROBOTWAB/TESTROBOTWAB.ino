// ════════════════════════════════════════════════════════════
//   robot_test_drive.cpp — Manual Drive Test (Web UI)
//
//   State Flow:
//     S_IDLE → (กด GO) → S_MOVE → S_AT_TARGET
//                                      ↓ (กด BACK)
//                                   S_BACK → S_IDLE
// ════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <math.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

ESP32Encoder wheel1, wheel2, wheel3;
MPU6050 mpu;

// ════════════════════════════════════
// CONFIG
// ════════════════════════════════════
const char* WIFI_SSID = "....."; ชื่อ wifi or your name wifi
const char* WIFI_PASS = "....."; Password your wifi

WebServer        httpServer(80);
WebSocketsServer wsServer(81);

// ── Pins ─────────────────────────────
#define Upper1  16
#define Lower1  17
#define Upper2  18
#define Lower2  19
#define Upper3  23
#define Lower3  25

#define ENC1_A  26
#define ENC1_B  27
#define ENC2_A  13
#define ENC2_B  14
#define ENC3_A  32
#define ENC3_B  33

#define LED_RED    4
#define LED_GREEN  2
#define LED_YELLOW 15

// ── Robot params ──────────────────────
#define PPR       17
#define N_GEAR    19
#define WHEEL_R   0.041f
#define ROBOT_L   0.2715f
#define DT        0.01f

const float PULSE_TO_RAD = (2.0f * PI) / (PPR * N_GEAR);
#define POS_TOLERANCE_RAD  0.5f

// ════════════════════════════════════
// IMU
// ════════════════════════════════════
float yaw = 0, gyroBiasZ = 0, targetYaw = 0;
unsigned long imuPrevTime = 0;

void calibrateGyro(int samples = 1000) {
    long sum = 0;
    int16_t ax, ay, az, gx, gy, gz;
    Serial.print("Calibrating gyro");
    for (int i = 0; i < samples; i++) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sum += gz;
        if (i % 100 == 0) Serial.print(".");
        delay(2);
    }
    gyroBiasZ = sum / (float)samples;
    Serial.printf("\nGyro bias Z: %.2f\n", gyroBiasZ);
}

void updateYaw() {
    unsigned long now = micros();
    float dt = (now - imuPrevTime) / 1e6f;
    imuPrevTime = now;
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float gz_dps = (gz - gyroBiasZ) / 65.5f;
    if (fabsf(gz_dps) < 0.3f) gz_dps = 0;
    yaw += gz_dps * (PI / 180.0f) * dt;
    if (yaw >  PI) yaw -= 2.0f * PI;
    if (yaw < -PI) yaw += 2.0f * PI;
}

// ════════════════════════════════════
// Shared vars
// ════════════════════════════════════
volatile long pos1, pos2, pos3;
volatile long last_pos1, last_pos2, last_pos3;
volatile bool isr_flag = false;
volatile bool running  = false;

float actual_pos1 = 0, actual_vel1 = 0;
float actual_pos2 = 0, actual_vel2 = 0;
float actual_pos3 = 0, actual_vel3 = 0;
float t_elapsed   = 0;
float theta_corr  = 0;

float target_vel1 = 0, target_vel2 = 0, target_vel3 = 0;
float duty1_out   = 0, duty2_out   = 0, duty3_out   = 0;
float target_pos1 = 0, target_pos2 = 0, target_pos3 = 0;

float active_target_x = 0.0f;
float active_target_y = 0.5f;

// ════════════════════════════════════
// Trapezoidal Profile
// ════════════════════════════════════
float prof_a = 0, prof_vmax = 0, prof_dist = 0, tacce = 0, sd = 0;
float cos_angle = 1, sin_angle = 0;

void profile_init(float tx, float ty, float acc) {
    prof_dist = sqrtf(tx*tx + ty*ty);
    float ang = atan2f(ty, tx);
    cos_angle = cosf(ang);
    sin_angle = sinf(ang);
    prof_a    = acc;
    prof_vmax = sqrtf(acc * prof_dist);
    tacce     = prof_vmax / acc;
    sd        = 0.5f * acc * tacce * tacce;
}

float profile_vel(float t) {
    if      (t <= tacce)         return prof_a * t;
    else if (t <= 2.0f * tacce)  return prof_vmax - prof_a * (t - tacce);
    else                         return 0.0f;
}

float profile_pos(float t) {
    if (t <= tacce) {
        return 0.5f * prof_a * t * t;
    } else if (t <= 2.0f * tacce) {
        float dt2 = t - tacce;
        return sd + prof_vmax*dt2 - 0.5f*prof_a*dt2*dt2;
    } else {
        return prof_dist;
    }
}

// ════════════════════════════════════
// PID
// ════════════════════════════════════
struct PID {
    float Kp, Ki, integral, limit;
    float update(float err) {
        integral = constrain(integral + err * DT, -limit, limit);
        return constrain(Kp * err + Ki * integral, -limit, limit);
    }
    void reset() { integral = 0; }
};

PID pid_pos1 = { 1.9489f, 0.0f, 0.0f, 40.0f };
PID pid_pos2 = { 1.9489f, 0.0f, 0.0f, 40.0f };
PID pid_pos3 = { 1.9489f, 0.0f, 0.0f, 40.0f };
PID pid_vel1 = { 0.67056f, 1.156f, 0.0f,  12.0f };
PID pid_vel2 = { 0.67056f, 1.156f, 0.0f,  12.0f };
PID pid_vel3 = { 0.67056f, 1.156f, 0.0f,  12.0f };
PID pid_yaw  = { 15.0f,  0.0f,  0.0f,  6.04f };

// ════════════════════════════════════
// Motor + LED
// ════════════════════════════════════
void motor_init() {
    ledcSetup(0,5000,8); ledcAttachPin(Upper1,0);
    ledcSetup(1,5000,8); ledcAttachPin(Lower1,1);
    ledcSetup(2,5000,8); ledcAttachPin(Upper2,2);
    ledcSetup(3,5000,8); ledcAttachPin(Lower2,3);
    ledcSetup(4,5000,8); ledcAttachPin(Upper3,4);
    ledcSetup(5,5000,8); ledcAttachPin(Lower3,5);
}

void set_motor(int rpwm, int lpwm, float duty) {
    duty = constrain(duty, -12.0f, 12.0f);
    uint8_t pwm = (uint8_t)((fabsf(duty)/12.0f)*255);
    if      (duty >  0.05f) { ledcWrite(rpwm,pwm); ledcWrite(lpwm,0);   }
    else if (duty < -0.05f) { ledcWrite(rpwm,0);   ledcWrite(lpwm,pwm); }
    else                    { ledcWrite(rpwm,0);   ledcWrite(lpwm,0);   }
}

void stop_all() { for(int i=0;i<6;i++) ledcWrite(i,0); }

void led_init() {
    pinMode(LED_RED,OUTPUT); pinMode(LED_GREEN,OUTPUT); pinMode(LED_YELLOW,OUTPUT);
    digitalWrite(LED_RED,LOW); digitalWrite(LED_GREEN,LOW); digitalWrite(LED_YELLOW,LOW);
}

void setLED(bool r, bool g, bool y) {
    digitalWrite(LED_RED,    r?HIGH:LOW);
    digitalWrite(LED_GREEN,  g?HIGH:LOW);
    digitalWrite(LED_YELLOW, y?HIGH:LOW);
}

uint32_t ledBlinkTime = 0;
bool     ledBlinkOn   = false;
void updateBlinkLED(int pin, uint32_t ms) {
    if (millis()-ledBlinkTime >= ms) {
        ledBlinkTime = millis();
        ledBlinkOn   = !ledBlinkOn;
        digitalWrite(pin, ledBlinkOn?HIGH:LOW);
    }
}

// ════════════════════════════════════
// Timer ISR
// ════════════════════════════════════
hw_timer_t*  timer    = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    pos1 = wheel1.getCount();
    pos2 = wheel2.getCount();
    pos3 = wheel3.getCount();

    actual_pos1 = pos1 * PULSE_TO_RAD;
    actual_pos2 = pos2 * PULSE_TO_RAD;
    actual_pos3 = pos3 * PULSE_TO_RAD;

    actual_vel1 = (pos1-last_pos1) * PULSE_TO_RAD / DT;
    actual_vel2 = (pos2-last_pos2) * PULSE_TO_RAD / DT;
    actual_vel3 = (pos3-last_pos3) * PULSE_TO_RAD / DT;

    last_pos1=pos1; last_pos2=pos2; last_pos3=pos3;
    if (running) t_elapsed += DT;
    isr_flag = true;
    portEXIT_CRITICAL_ISR(&timerMux);
}

// ════════════════════════════════════
// Utility
// ════════════════════════════════════
void reset_motion() {
    portENTER_CRITICAL(&timerMux);
    t_elapsed   = 0;
    target_vel1 = target_vel2 = target_vel3 = 0;
    duty1_out   = duty2_out   = duty3_out   = 0;
    pos1=pos2=pos3=0; last_pos1=last_pos2=last_pos3=0;
    actual_pos1=actual_pos2=actual_pos3=0;
    actual_vel1=actual_vel2=actual_vel3=0;
    target_pos1=target_pos2=target_pos3=0;
    yaw=0; targetYaw=0;
    wheel1.clearCount(); wheel2.clearCount(); wheel3.clearCount();
    portEXIT_CRITICAL(&timerMux);
    pid_pos1.reset(); pid_pos2.reset(); pid_pos3.reset();
    pid_vel1.reset(); pid_vel2.reset(); pid_vel3.reset();
    pid_yaw.reset();
}

bool is_position_reached() {
    return fabsf(target_pos1-actual_pos1) < POS_TOLERANCE_RAD &&
           fabsf(target_pos2-actual_pos2) < POS_TOLERANCE_RAD &&
           fabsf(target_pos3-actual_pos3) < POS_TOLERANCE_RAD;
}

// ════════════════════════════════════
// State Machine
// ════════════════════════════════════
enum State { S_IDLE, S_MOVE, S_AT_TARGET, S_BACK };
State state = S_IDLE;

// ════════════════════════════════════
// Web UI
// ════════════════════════════════════
const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html lang="th"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot Drive Test</title>
<style>
:root{--bg:#0d0d0d;--card:#1a1a1a;--border:#2a2a2a;--green:#00ff88;--yellow:#ffd700;--red:#ff4444;--blue:#4488ff;--text:#e0e0e0;--dim:#666}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:'Courier New',monospace;padding:16px;max-width:480px;margin:auto}
h1{color:var(--green);font-size:1rem;letter-spacing:3px;text-align:center;padding:10px 0 18px}

/* State badge */
.badge{text-align:center;padding:10px;border-radius:8px;border:1px solid var(--border);font-size:0.85rem;letter-spacing:2px;margin-bottom:16px;transition:all 0.3s}
.badge.idle      {color:var(--dim);   border-color:var(--border)}
.badge.moving    {color:var(--green); border-color:var(--green);  box-shadow:0 0 8px #00ff8833}
.badge.at-target {color:var(--yellow);border-color:var(--yellow); box-shadow:0 0 8px #ffd70033}
.badge.back      {color:var(--red);   border-color:var(--red);    box-shadow:0 0 8px #ff444433}

/* Coordinate input */
.coord-row{display:flex;gap:10px;margin-bottom:12px}
.coord-box{flex:1;background:var(--card);border:1px solid var(--border);border-radius:8px;padding:12px}
.coord-box label{display:block;font-size:0.65rem;color:var(--dim);letter-spacing:2px;margin-bottom:6px}
.coord-box input{width:100%;background:transparent;border:none;color:var(--text);font-family:'Courier New',monospace;font-size:1.5rem;text-align:center;outline:none}
.coord-box input:focus{color:var(--green)}
.coord-unit{font-size:0.65rem;color:var(--dim);text-align:center;margin-top:2px}

/* Buttons */
.btn-row{display:flex;gap:10px;margin-bottom:16px}
.btn{flex:1;border:none;border-radius:8px;padding:16px;font-family:'Courier New',monospace;font-size:0.9rem;font-weight:bold;letter-spacing:2px;cursor:pointer;transition:all 0.15s;opacity:1}
.btn:disabled{opacity:0.25;cursor:not-allowed}
.btn-go  {background:var(--green); color:#000}
.btn-back{background:var(--red);   color:#fff}
.btn-stop{background:#333;         color:var(--dim);border:1px solid var(--border);font-size:0.75rem;padding:10px;flex:0 0 60px}
.btn-go:not(:disabled):hover  {background:#00cc6e;box-shadow:0 0 12px #00ff8844}
.btn-back:not(:disabled):hover{background:#cc2222;box-shadow:0 0 12px #ff444444}

/* Data cards */
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:12px}
.card{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:12px}
.card h2{font-size:0.6rem;color:var(--dim);letter-spacing:2px;margin-bottom:6px}
.val{font-size:1.3rem;font-weight:bold}
.val.g{color:var(--green)}.val.y{color:var(--yellow)}.val.b{color:var(--blue)}.val.r{color:var(--red)}
.unit{font-size:0.65rem;color:var(--dim);margin-left:3px}
.full{grid-column:1/-1}

/* Encoder result panel */
.result{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:12px;margin-bottom:12px;display:none}
.result.show{display:block}
.result h2{font-size:0.6rem;color:var(--dim);letter-spacing:2px;margin-bottom:10px}
.result-row{display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid var(--border);font-size:0.8rem}
.result-row:last-child{border-bottom:none}
.result-label{color:var(--dim)}
.result-val{color:var(--green);font-weight:bold}

/* Chart */
.chart-wrap{margin-bottom:12px}
.chart-label{font-size:0.6rem;color:var(--dim);letter-spacing:2px;padding:8px 10px 0;background:var(--card);border-radius:8px 8px 0 0;border:1px solid var(--border);border-bottom:none}
canvas{width:100%;display:block;background:var(--card);border:1px solid var(--border);border-top:none;border-radius:0 0 8px 8px}
.legend{display:flex;gap:10px;padding:5px 10px 7px;background:var(--card);border:1px solid var(--border);border-top:none;border-radius:0 0 8px 8px;flex-wrap:wrap}
.legend span{font-size:0.6rem;display:flex;align-items:center;gap:4px}
.legend span::before{content:'';display:inline-block;width:10px;height:2px}
.l1::before{background:#00ff88}.l2::before{background:#ffd700}.l3::before{background:#4488ff}.l4::before{background:#ff4444}

/* Status */
.status{display:flex;align-items:center;gap:8px;font-size:0.7rem;justify-content:center;color:var(--dim)}
.dot{width:7px;height:7px;border-radius:50%;background:var(--red)}
.dot.on{background:var(--green);animation:p 1s infinite}
@keyframes p{0%,100%{opacity:1}50%{opacity:.4}}
</style></head><body>

<h1>◈ ROBOT DRIVE TEST</h1>

<div class="badge idle" id="badge">○  IDLE</div>

<div class="coord-row">
  <div class="coord-box">
    <label>X TARGET</label>
    <input type="number" id="tx" step="0.1" value="0.0">
    <div class="coord-unit">meter</div>
  </div>
  <div class="coord-box">
    <label>Y TARGET</label>
    <input type="number" id="ty" step="0.1" value="0.5">
    <div class="coord-unit">meter</div>
  </div>
</div>

<div class="btn-row">
  <button class="btn btn-go"   id="btnGo"   onclick="cmdGo()">▶  GO</button>
  <button class="btn btn-back" id="btnBack" onclick="cmdBack()" disabled>◀  BACK</button>
  <button class="btn btn-stop"              onclick="cmdStop()">■</button>
</div>

<!-- ผลลัพธ์ encoder หลังกด BACK -->
<div class="result" id="resultPanel">
  <h2>ENCODER RESULT (after back)</h2>
  <div class="result-row"><span class="result-label">Wheel 1</span><span class="result-val" id="r1">—</span></div>
  <div class="result-row"><span class="result-label">Wheel 2</span><span class="result-val" id="r2">—</span></div>
  <div class="result-row"><span class="result-label">Wheel 3</span><span class="result-val" id="r3">—</span></div>
  <div class="result-row"><span class="result-label">Yaw</span>    <span class="result-val" id="ry">—</span></div>
</div>

<div class="grid">
  <div class="card"><h2>YAW</h2>           <div class="val y" id="yaw">0.00<span class="unit">deg</span></div></div>
  <div class="card"><h2>YAW CORRECTION</h2><div class="val b" id="corr">0.000<span class="unit">r/s</span></div></div>
  <div class="card"><h2>WHEEL 1</h2>       <div class="val g" id="p1">0.000<span class="unit">rad</span></div></div>
  <div class="card"><h2>WHEEL 2</h2>       <div class="val g" id="p2">0.000<span class="unit">rad</span></div></div>
  <div class="card full"><h2>WHEEL 3</h2>  <div class="val g" id="p3">0.000<span class="unit">rad</span></div></div>
</div>

<div class="chart-wrap">
  <div class="chart-label">WHEEL POSITION</div>
  <canvas id="cv" height="90"></canvas>
  <div class="legend"><span class="l1">wheel 1</span><span class="l2">wheel 2</span><span class="l3">wheel 3</span></div>
</div>

<div class="status"><div class="dot" id="dot"></div><span id="ct">Connecting...</span></div>

<script>
const MAX=120;
let cdata=[[],[],[]];
const colors=['#00ff88','#ffd700','#4488ff'];
let cvs,ctx;

function initChart(){
  cvs=document.getElementById('cv');
  ctx=cvs.getContext('2d');
  cvs.width=cvs.offsetWidth;
}

function pushChart(v){
  v.forEach((x,i)=>{
    cdata[i].push(parseFloat(x)||0);
    if(cdata[i].length>MAX) cdata[i].shift();
  });
  let mn=Infinity,mx=-Infinity;
  cdata.forEach(d=>d.forEach(v=>{if(v<mn)mn=v;if(v>mx)mx=v}));
  const range=mx-mn||1,pad=6,w=cvs.width,h=cvs.height;
  ctx.clearRect(0,0,w,h);
  ctx.fillStyle='#1a1a1a'; ctx.fillRect(0,0,w,h);
  ctx.strokeStyle='#2a2a2a'; ctx.lineWidth=0.5;
  [1,2,3].forEach(i=>{ctx.beginPath();ctx.moveTo(0,h/4*i);ctx.lineTo(w,h/4*i);ctx.stroke()});
  cdata.forEach((d,i)=>{
    if(!d.length) return;
    ctx.beginPath(); ctx.strokeStyle=colors[i]; ctx.lineWidth=1.5;
    d.forEach((v,j)=>{
      const x=j/(MAX-1)*w, y=pad+(1-(v-mn)/range)*(h-pad*2);
      j===0?ctx.moveTo(x,y):ctx.lineTo(x,y);
    });
    ctx.stroke();
  });
}

// State UI
const states = {
  IDLE:      {cls:'idle',      txt:'○  IDLE',          go:true,  back:false},
  MOVE:      {cls:'moving',    txt:'▶  MOVING...',      go:false, back:false},
  AT_TARGET: {cls:'at-target', txt:'◉  AT TARGET',      go:false, back:true},
  BACK:      {cls:'back',      txt:'◀  RETURNING...',   go:false, back:false},
};

function applyState(s){
  const m = states[s] || states.IDLE;
  const b = document.getElementById('badge');
  b.className = 'badge '+m.cls;
  b.textContent = m.txt;
  document.getElementById('btnGo').disabled   = !m.go;
  document.getElementById('btnBack').disabled = !m.back;
}

let ws;
function connect(){
  ws = new WebSocket('ws://'+location.hostname+':81');
  ws.onopen  = ()=>{ document.getElementById('dot').className='dot on'; document.getElementById('ct').textContent='Connected'; };
  ws.onclose = ()=>{ document.getElementById('dot').className='dot';    document.getElementById('ct').textContent='Disconnected — retrying...'; setTimeout(connect,2000); };
  ws.onmessage = (e)=>{
    const d = e.data;

    // STATE packet
    if(d.startsWith('STATE:')){
      applyState(d.substring(6));
      return;
    }

    // RESULT packet: RESULT:p1,p2,p3,yaw
    if(d.startsWith('RESULT:')){
      const v = d.substring(7).split(',');
      document.getElementById('r1').textContent = parseFloat(v[0]).toFixed(4)+' rad';
      document.getElementById('r2').textContent = parseFloat(v[1]).toFixed(4)+' rad';
      document.getElementById('r3').textContent = parseFloat(v[2]).toFixed(4)+' rad';
      document.getElementById('ry').textContent = parseFloat(v[3]).toFixed(2)+' deg';
      document.getElementById('resultPanel').className='result show';
      return;
    }

    // DATA packet: p1,p2,p3,yaw,corr,target
    const v = d.split(',');
    if(v.length < 6) return;
    document.getElementById('p1').innerHTML  = parseFloat(v[0]).toFixed(3)+'<span class="unit">rad</span>';
    document.getElementById('p2').innerHTML  = parseFloat(v[1]).toFixed(3)+'<span class="unit">rad</span>';
    document.getElementById('p3').innerHTML  = parseFloat(v[2]).toFixed(3)+'<span class="unit">rad</span>';
    document.getElementById('yaw').innerHTML = parseFloat(v[3]).toFixed(2)+'<span class="unit">deg</span>';
    document.getElementById('corr').innerHTML= parseFloat(v[4]).toFixed(3)+'<span class="unit">r/s</span>';
    pushChart([v[0],v[1],v[2]]);
  };
}

function cmdGo(){
  const x = parseFloat(document.getElementById('tx').value)||0;
  const y = parseFloat(document.getElementById('ty').value)||0;
  if(ws && ws.readyState===1) ws.send('GO:'+x+','+y);
}
function cmdBack(){
  if(ws && ws.readyState===1) ws.send('BACK');
}
function cmdStop(){
  if(ws && ws.readyState===1) ws.send('STOP');
}

initChart(); connect();
</script>
</body></html>
)rawhtml";

// ── WebSocket handler ─────────────────
void wsSendState(const char* s) {
    char buf[24];
    snprintf(buf, sizeof(buf), "STATE:%s", s);
    wsServer.broadcastTXT(buf);
}

void wsSendResult() {
    char buf[80];
    snprintf(buf, sizeof(buf), "RESULT:%.4f,%.4f,%.4f,%.2f",
             actual_pos1, actual_pos2, actual_pos3,
             yaw * (180.0f / PI));
    wsServer.broadcastTXT(buf);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    if (type != WStype_TEXT) return;
    String msg = String((char*)payload);

    // ── STOP ────────────────────────────
    if (msg == "STOP") {
        portENTER_CRITICAL(&timerMux); running = false; portEXIT_CRITICAL(&timerMux);
        stop_all();
        reset_motion();
        state = S_IDLE;
        setLED(false, false, true);
        wsSendState("IDLE");
        Serial.println("[WS] STOP");
        return;
    }

    // ── GO ──────────────────────────────
    if (msg.startsWith("GO:") && state == S_IDLE) {
        String data = msg.substring(3);
        int comma = data.indexOf(',');
        if (comma <= 0) return;
        float nx = data.substring(0, comma).toFloat();
        float ny = data.substring(comma + 1).toFloat();
        if (sqrtf(nx*nx + ny*ny) > 3.0f) { Serial.println("[WS] Too far"); return; }

        active_target_x = nx;
        active_target_y = ny;

        reset_motion();
        profile_init(active_target_x, active_target_y, 3.0f);
        portENTER_CRITICAL(&timerMux); running = true; portEXIT_CRITICAL(&timerMux);
        state = S_MOVE;
        setLED(false, true, false);
        wsSendState("MOVE");
        Serial.printf("[WS] GO x=%.3f y=%.3f\n", active_target_x, active_target_y);
        return;
    }

    // ── BACK ─────────────────────────────
    if (msg == "BACK" && state == S_AT_TARGET) {
        // บันทึกค่า encoder ก่อนกลับ
        Serial.println("════ SNAPSHOT (before back) ════");
        Serial.printf("pos1: %.4f rad\n", actual_pos1);
        Serial.printf("pos2: %.4f rad\n", actual_pos2);
        Serial.printf("pos3: %.4f rad\n", actual_pos3);
        Serial.printf("yaw:  %.2f deg\n",  yaw * (180.0f / PI));
        Serial.println("════════════════════════════════");

        reset_motion();
        profile_init(-active_target_x, -active_target_y, 3.0f);
        portENTER_CRITICAL(&timerMux); running = true; portEXIT_CRITICAL(&timerMux);
        state = S_BACK;
        setLED(true, false, false);
        wsSendState("BACK");
        Serial.println("[WS] BACK");
        return;
    }
}

// ── WiFi init ─────────────────────────
void wifi_init() {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("WiFi connecting");
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 30) {
        delay(500); Serial.print("."); tries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n✓ WiFi IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n✗ AP fallback");
        WiFi.softAP("RobotTest", "robot1234");
        Serial.printf("AP: %s\n", WiFi.softAPIP().toString().c_str());
    }
    httpServer.on("/", []() { httpServer.send_P(200, "text/html", INDEX_HTML); });
    httpServer.begin();
    wsServer.begin();
    wsServer.onEvent(webSocketEvent);
    Serial.println("✓ HTTP + WebSocket ready");
}

void wifi_tick() {
    httpServer.handleClient();
    wsServer.loop();
}

// ════════════════════════════════════
// Setup
// ════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("════ BOOT ════");

    led_init(); setLED(true,true,true);

    wheel1.attachFullQuad(ENC1_A,ENC1_B);
    wheel2.attachFullQuad(ENC2_A,ENC2_B);
    wheel3.attachFullQuad(ENC3_A,ENC3_B);
    wheel1.clearCount(); wheel2.clearCount(); wheel3.clearCount();
    Serial.println("✓ Encoder");

    motor_init(); stop_all();
    Serial.println("✓ Motor");

    Wire.begin(21,22); delay(200);
    Wire.setClock(400000);
    mpu.initialize();
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    calibrateGyro(1000);
    targetYaw   = 0;
    imuPrevTime = micros();
    Serial.println("✓ IMU");

    wifi_init();

    profile_init(0.0f, 0.0f, 3.0f);

    timer = timerBegin(0,80,true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer,10000,true);
    timerAlarmEnable(timer);
    Serial.println("✓ Timer 100 Hz");

    state = S_IDLE;
    setLED(false, false, true);
    Serial.println("════ READY ════");
    Serial.printf("http://%s\n", WiFi.localIP().toString().c_str());
}

// ════════════════════════════════════
// Loop
// ════════════════════════════════════
void loop() {
    if (!isr_flag) return;
    isr_flag = false;

    static uint8_t cnt = 0;
    cnt++;

    // ── 1. Velocity PID (10 ms) ──────────────
    bool is_running;
    portENTER_CRITICAL(&timerMux); is_running = running; portEXIT_CRITICAL(&timerMux);

    duty1_out = pid_vel1.update(target_vel1 - actual_vel1) * is_running;
    duty2_out = pid_vel2.update(target_vel2 - actual_vel2) * is_running;
    duty3_out = pid_vel3.update(target_vel3 - actual_vel3) * is_running;
    set_motor(0,1,duty1_out);
    set_motor(2,3,duty2_out);
    set_motor(4,5,duty3_out);

    // ── 2. Profile + IK + IMU (50 ms) ────────
    if (cnt % 5 == 0) {
        float local_t;
        portENTER_CRITICAL(&timerMux); local_t = t_elapsed; portEXIT_CRITICAL(&timerMux);

        float vel = profile_vel(local_t);
        float pos = profile_pos(local_t);
        float vx  = vel * cos_angle, vy = vel * sin_angle;
        float px  = pos * cos_angle, py = pos * sin_angle;

        updateYaw();
        theta_corr = pid_yaw.update(targetYaw - yaw);

        float w1 = ( 0.000f*vx - 1.000f*vy + theta_corr*ROBOT_L) / WHEEL_R;
        float w2 = ( 0.866f*vx + 0.500f*vy + theta_corr*ROBOT_L) / WHEEL_R;
        float w3 = (-0.866f*vx + 0.500f*vy + theta_corr*ROBOT_L) / WHEEL_R;

        target_pos1 = ( 0.000f*px - 1.000f*py) / WHEEL_R;
        target_pos2 = ( 0.866f*px + 0.500f*py) / WHEEL_R;
        target_pos3 = (-0.866f*px + 0.500f*py) / WHEEL_R;

        target_vel1 = w1 + pid_pos1.update(target_pos1 - actual_pos1);
        target_vel2 = w2 + pid_pos2.update(target_pos2 - actual_pos2);
        target_vel3 = w3 + pid_pos3.update(target_pos3 - actual_pos3);
    }

    // ── 3. Serial + WiFi (100 ms) ─────────────
    if (cnt % 10 == 0) {
        cnt = 0;
        float yd = yaw * (180.0f / PI);
        float td = targetYaw * (180.0f / PI);

        Serial.printf("%.3f,%.3f,%.3f,%.2f,%.3f\n",
            actual_pos1, actual_pos2, actual_pos3, yd, theta_corr);

        char buf[80];
        snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f,%.2f,%.3f,%.2f",
                 actual_pos1, actual_pos2, actual_pos3, yd, theta_corr, td);
        wsServer.broadcastTXT(buf);

        wifi_tick();
    } else {
        wifi_tick();
    }

    // ── 4. State Machine ─────────────────────
    float current_t;
    portENTER_CRITICAL(&timerMux); current_t = t_elapsed; portEXIT_CRITICAL(&timerMux);

    switch (state) {

        case S_IDLE:
            stop_all();
            target_vel1 = target_vel2 = target_vel3 = 0;
            break;

        case S_MOVE:
            updateBlinkLED(LED_GREEN, 200);
            if ((current_t >= 2.0f * tacce) &&
                (is_position_reached() || current_t >= 2.0f * tacce + 1.0f)) {
                stop_all();
                portENTER_CRITICAL(&timerMux); running = false; portEXIT_CRITICAL(&timerMux);
                state = S_AT_TARGET;
                setLED(false, false, true);
                wsSendState("AT_TARGET");
                Serial.println("[SM] MOVE → AT_TARGET  (รอกด BACK)");
            }
            break;

        case S_AT_TARGET:
            // หยุดรอคำสั่ง BACK จาก Web UI
            stop_all();
            target_vel1 = target_vel2 = target_vel3 = 0;
            updateBlinkLED(LED_YELLOW, 500);
            break;

        case S_BACK:
            updateBlinkLED(LED_RED, 200);
            if ((current_t >= 2.0f * tacce) &&
                (is_position_reached() || current_t >= 2.0f * tacce + 1.0f)) {
                stop_all();
                portENTER_CRITICAL(&timerMux); running = false; portEXIT_CRITICAL(&timerMux);

                // ส่งค่า encoder ขณะ BACK เสร็จไป Web
                wsSendResult();

                Serial.println("════ RESULT (after back) ════");
                Serial.printf("pos1: %.4f rad\n", actual_pos1);
                Serial.printf("pos2: %.4f rad\n", actual_pos2);
                Serial.printf("pos3: %.4f rad\n", actual_pos3);
                Serial.printf("yaw:  %.2f deg\n",  yaw*(180.0f/PI));
                Serial.println("══════════════════════════════");

                reset_motion();
                state = S_IDLE;
                setLED(false, false, true);
                wsSendState("IDLE");
                Serial.println("[SM] BACK → IDLE");
            }
            break;
    }
}
