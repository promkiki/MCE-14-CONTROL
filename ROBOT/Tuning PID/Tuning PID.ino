#include <Arduino.h>
#include <math.h>
#include <ESP32Encoder.h>
ESP32Encoder wheel1, wheel2, wheel3;
// ════════════════════════════════════
// TEST MODE
// 1 = เช็ค encoder
// 2 = เช็ค motor ทิศทาง
// 3 = เช็ค profile
// 4 = เช็ค PID ล้อเดี่ยว
// 5 = รันทั้งระบบ
// ════════════════════════════════════
#define TEST_MODE 5

// ── Pins ─────────────────────────────
#define Upper1  16
#define Lower1  17
#define Upper2  18
#define Lower2  19
#define Upper3  23
#define Lower3  25
// ── Encoder ───────────────────────
#define ENC1_A  26
#define ENC1_B  27
#define ENC2_A  13
#define ENC2_B  14
#define ENC3_A  32
#define ENC3_B  33

// ── Robot params ─────────────────────
#define PPR      17
#define N_GEAR   19
#define WHEEL_R  0.041f
#define ROBOT_L  0.2715f
#define DT       0.01f

const float PULSE_TO_RAD = (2.0f * PI) / (PPR * N_GEAR);


volatile long pos1, pos2, pos3;

volatile long last_pos1, last_pos2, last_pos3;


// ── Encoder ──────────────────────────
volatile int32_t pulse1 = 0;
volatile int32_t pulse2 = 0;
volatile int32_t pulse3 = 0;



// ── Shared vars ───────────────────────
volatile bool  isr_flag  = false;
volatile bool  running   = false;

float actual_pos1 = 0, actual_vel1 = 0;
float actual_pos2 = 0, actual_vel2 = 0;
float actual_pos3 = 0, actual_vel3 = 0;
float t_elapsed   = 0;
float actual_ms =0;

float target_vel1 = 0;
float target_vel2 = 0;
float target_vel3 = 0;

float duty1_out = 0;
float duty2_out = 0;
float duty3_out = 0;

// ── Profile ──────────────────────────
float prof_a    = 0;
float prof_vmax = 0;
float prof_dist = 0;
float tacce     = 0;
float sd        = 0;
float cos_angle = 1;
float sin_angle = 0;

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
    else if (t <= 2.0f * tacce)  return prof_vmax - prof_a*(t - tacce);
    else                         return 0.0f;
}

float profile_pos(float t) {
    if (t <= tacce) {
        return 0.5f * prof_a * t * t;
    } else if (t <= 2.0f * tacce) {
        float dt = t - tacce;
        return sd + prof_vmax*dt - 0.5f*prof_a*dt*dt;
    } else {
        return prof_dist;
    }
}

// ── PID ──────────────────────────────
struct PID {
    float Kp, Ki, integral, limit;
    float update(float err) {
        integral = constrain(integral + err * DT, -limit, limit);
        return constrain(Kp * err + Ki * integral, -limit, limit);
    }
    void reset() { integral = 0; }
};

//            Kp      Ki    integral  limit
PID pid_pos1 = { 1.9489f, 0.0f, 0.0f, 40.0f };
PID pid_pos2 = { 1.9489f, 0.0f, 0.0f, 40.0f };
PID pid_pos3 = { 1.9489f, 0.0f, 0.0f, 40.0f };

PID pid_vel1 = { 0.67056f, 1.156f, 0.0f,  12.0f };
PID pid_vel2 = { 0.67056f, 1.156f, 0.0f,  12.0f };
PID pid_vel3 = { 0.67056f, 1.156f, 0.0f,  12.0f };

// ── Motor ─────────────────────────────
void motor_init() {
    ledcSetup(0, 5000, 8); ledcAttachPin(Upper1, 0);
    ledcSetup(1, 5000, 8); ledcAttachPin(Lower1, 1);
    ledcSetup(2, 5000, 8); ledcAttachPin(Upper2, 2);
    ledcSetup(3, 5000, 8); ledcAttachPin(Lower2, 3);
    ledcSetup(4, 5000, 8); ledcAttachPin(Upper3, 4);
    ledcSetup(5, 5000, 8); ledcAttachPin(Lower3, 5);
}

// ← ledcWrite อยู่ที่นี่เท่านั้น ไม่เข้า ISR
void set_motor(int rpwm, int lpwm, float duty) {
    
    duty = constrain(duty, -12.0f, 12.0f);
    uint8_t pwm = (uint8_t)((fabsf(duty) /12)* 255);
    
    if (duty > 0.05f) {
        ledcWrite(rpwm, pwm);
        ledcWrite(lpwm, 0);
    } else if (duty < -0.05f) {
        ledcWrite(rpwm, 0);
        ledcWrite(lpwm, pwm);
    } else {
        ledcWrite(rpwm, 0);
        ledcWrite(lpwm, 0);
    }

}

void stop_all() {
    for (int i = 0; i < 6; i++) ledcWrite(i, 0);
}

// ── State machine ─────────────────────
enum State { S_MOVE, S_WAIT, S_BACK, S_CHECK };
State    state  = S_MOVE;
uint32_t t_wait = 0;

void reset_all() {
    t_elapsed   = 0;
    target_vel1 = 0;  target_vel2 = 0;  target_vel3 = 0;
    duty1_out   = 0;  duty2_out   = 0;  duty3_out   = 0;
    pos1 = 0;  pos2 = 0 ; pos3 = 0;
    last_pos1 = 0; last_pos2 = 0; last_pos3 = 0;
    actual_pos1 = 0; actual_pos2 = 0; actual_pos3 =0;
    actual_vel1 = 0; actual_vel2 = 0; actual_vel3 =0;
    wheel1.clearCount();
    wheel2.clearCount();
    wheel3.clearCount();
    pid_pos1.reset(); pid_pos2.reset(); pid_pos3.reset();
    pid_vel1.reset(); pid_vel2.reset(); pid_vel3.reset();
}

void do_move() {
    if (t_elapsed >= 2.0f * tacce) {
        stop_all();
        running = false;
        t_wait  = millis();
        state   = S_WAIT;
        Serial.println("→ WAIT");
    }
}

void do_wait() {
    if (millis() - t_wait >= 2000) {
        reset_all();
        profile_init(-0.8f, -0.0f, 3.0f);
        running = true;
        state   = S_BACK;
        Serial.println("→ BACK");
    }
}

void do_back() {
    if (t_elapsed >= 2.0f * tacce) {
        stop_all();
        running = false;
        state   = S_CHECK;
        Serial.println("→ CHECK");
    }
}

void do_check() {
    Serial.println("════ RESULT ════");
    Serial.print("pos1: "); Serial.println(actual_pos1, 4);
    Serial.print("pos2: "); Serial.println(actual_pos2, 4);
    Serial.print("pos3: "); Serial.println(actual_pos3, 4);
    Serial.println("════════════════");
    delay(5000);

    reset_all();
    profile_init(0.80f, 0.00f, 3.0f);
    running = true;
    state   = S_MOVE;
    Serial.println("→ MOVE");
}

// ── Timer ─────────────────────────────
hw_timer_t*  timer    = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ─────────────────────────────────────
// ISR — อ่าน encoder อย่างเดียว
// ไม่มี ledcWrite เด็ดขาด
// ─────────────────────────────────────
void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);

    static int32_t p1=0, p2=0, p3=0;
    int32_t c1=pulse1, c2=pulse2, c3=pulse3;
    pos1 = wheel1.getCount();
    pos2 = wheel2.getCount();
    pos3 = wheel3.getCount();
    
    actual_pos1 = pos1* PULSE_TO_RAD;
    actual_pos2 = pos2 * PULSE_TO_RAD;
    actual_pos3 = pos3 * PULSE_TO_RAD;


    actual_vel1 =  (pos1 - last_pos1) * PULSE_TO_RAD / DT;
    actual_vel2 = (pos2 - last_pos2)* PULSE_TO_RAD / DT;
    actual_vel3 = (pos3 - last_pos3) * PULSE_TO_RAD / DT;
    actual_ms = actual_vel1*WHEEL_R;

    last_pos1 = pos1;
    last_pos2 = pos2;
    last_pos3 = pos3;   
    t_elapsed += DT * running;


    isr_flag = true;
    portEXIT_CRITICAL_ISR(&timerMux);
}

// ─────────────────────────────────────
// Setup
// ─────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("════ BOOT ════");

    wheel1.attachFullQuad(26, 27);
    wheel2.attachFullQuad(13, 14);
    wheel3.attachFullQuad(32, 33);
    Serial.println("✓ Encoder");
    
    wheel1.clearCount();
    wheel2.clearCount();
    wheel3.clearCount();

    motor_init();
    stop_all();
    Serial.println("✓ Motor");

    profile_init(0.5f, 0.0f, 3.0f);
    Serial.println("✓ Profile");
    Serial.print("  dist : "); Serial.println(prof_dist,  4);
    Serial.print("  vmax : "); Serial.println(prof_vmax,  4);
    Serial.print("  tacce: "); Serial.println(tacce,      4);
    Serial.print("  ttot : "); Serial.println(2.0f*tacce, 4);

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 10000, true);
    timerAlarmEnable(timer);
    Serial.println("✓ Timer");

    Serial.print("TEST_MODE = ");
    Serial.println(TEST_MODE);
    Serial.println("══════════════");

    #if TEST_MODE == 5
        running = true;
    #endif
}

// ─────────────────────────────────────
// Loop — ledcWrite อยู่ที่นี่เท่านั้น
// ─────────────────────────────────────
void loop() {

// ════════════════════════════════════
// TEST 1 — encoder
// หมุนล้อ 1 รอบ → pulse = 76
// ════════════════════════════════════
#if TEST_MODE == 1
    static uint32_t last = 0;
    if (millis() - last >= 200) {
        last = millis();
        Serial.print("p1:"); Serial.print(pos1);
        Serial.print("\tp2:"); Serial.print(pos2);
        Serial.print("\tp3:"); Serial.print(pos3);
        Serial.print("\tp1d:"); Serial.print(actual_pos1);
        Serial.print("\tp2d:"); Serial.print(actual_pos2);
        Serial.print("\tp3d:"); Serial.print(actual_pos3);
         Serial.print("\tp3Ws:"); Serial.print(actual_vel1);
        Serial.print("\tp1Ws:"); Serial.print(actual_vel2);
        Serial.print("\tp2Ws:"); Serial.println(actual_vel3);
    
    
    }

// ════════════════════════════════════
// TEST 2 — motor ทิศทาง
// ดูว่าล้อหมุนถูกทิศไหม
// ════════════════════════════════════
#elif TEST_MODE == 2
    Serial.println("M1 forward");
    set_motor(0, 1,  0.3f); delay(2000);
    stop_all();              delay(500);

    Serial.println("M1 backward");
    set_motor(0, 1, -0.3f); delay(2000);
    stop_all();              delay(500);

    Serial.println("M2 forward");
    set_motor(2, 3,  0.3f); delay(2000);
    stop_all();              delay(500);

    Serial.println("M2 backward");
    set_motor(2, 3, -0.3f); delay(2000);
    stop_all();              delay(500);

    Serial.println("M3 forward");
    set_motor(4, 5,  0.3f); delay(2000);
    stop_all();              delay(500);

    Serial.println("M3 backward");
    set_motor(4, 5, -0.3f); delay(2000);
    stop_all();

    Serial.println("DONE");
    delay(999999);

// ════════════════════════════════════
// TEST 3 — profile
// เปิด Serial Plotter
// บรรทัด 1 = velocity
// บรรทัด 2 = position
// ════════════════════════════════════
#elif TEST_MODE == 3
    static float t = 0;
    if (t <= 2.0f * tacce + 0.05f) {
        Serial.print(profile_vel(t), 4);
        Serial.print(",");
        Serial.println(profile_pos(t), 4);
        t += 0.005f;
    }
    delay(5);

// ════════════════════════════════════
// TEST 4 — PID ล้อเดี่ยว
// เปิด Serial Plotter
// บรรทัด 1 = target
// บรรทัด 2 = actual vel
// ════════════════════════════════════
#elif TEST_MODE == 4
    if (!isr_flag) return;
    isr_flag = false;

    float target = 15.0f;   // rad/s ปรับได้
    duty1_out = pid_vel1.update(target - actual_vel1);

    set_motor(0, 1, duty1_out);
    set_motor(2, 3, 0);
    set_motor(4, 5, 0);

    static uint32_t last = 0;
    if (millis() - last >= 10) {
        last = millis();
        Serial.print(target);
        Serial.print(",");
        Serial.print(actual_vel1, 3);
        Serial.print(",");
        Serial.print(actual_ms, 3);
        Serial.print(",");
        Serial.println(duty1_out);
    }

// ════════════════════════════════════
// TEST 5 — ทั้งระบบ
// เปิด Serial Plotter
// บรรทัด 1,2,3 = pos แต่ละล้อ
// ════════════════════════════════════
#elif TEST_MODE == 5
    if (!isr_flag) return;
    isr_flag = false;

    static uint8_t cnt = 0;
    cnt++;

    // inner PID ทุก 1ms
    duty1_out = pid_vel1.update(target_vel1 - actual_vel1) * running;
    duty2_out = pid_vel2.update(target_vel2 - actual_vel2) * running;
    duty3_out = pid_vel3.update(target_vel3 - actual_vel3) * running;

    // ledcWrite ที่นี่เท่านั้น
    set_motor(0, 1, duty1_out);
    set_motor(2, 3, duty2_out);
    set_motor(4, 5, duty3_out);

    // outer PID + profile ทุก 5ms
    if (cnt % 5 == 0) {
        float vel = profile_vel(t_elapsed);
        float pos = profile_pos(t_elapsed);

        float vx = vel * cos_angle;
        float vy = vel * sin_angle;
        float px = pos * cos_angle;
        float py = pos * sin_angle;

        float w1 = ( 0.000f*vx - 1.000f*vy) / WHEEL_R;
        float w2 = (0.866f*vx + 0.500f*vy) / WHEEL_R;
        float w3 = (-0.866f*vx + 0.500f*vy) / WHEEL_R;

        float p1 = ( 0.000f*px - 1.000f*py) / WHEEL_R;
        float p2 =  (0.866f*px + 0.500f*py) / WHEEL_R;
        float p3 = ( -0.866f*px + 0.500f*py) / WHEEL_R;

        target_vel1 = w1 + pid_pos1.update(p1 - actual_pos1);
        target_vel2 = w2 + pid_pos2.update(p2 - actual_pos2);
        target_vel3 = w3 + pid_pos3.update(p3 - actual_pos3);
    }

    // serial ทุก 100ms
    if (cnt % 10 == 0) {
        cnt = 0;
        Serial.print(actual_pos1, 3); Serial.print(",");
        Serial.print(actual_pos2, 3); Serial.print(",");
        Serial.print(actual_pos3, 3);
        Serial.print(duty1_out, 3); Serial.print(",");
        Serial.print(duty2_out, 3); Serial.print(",");
        Serial.print(duty3_out, 3);
        Serial.print(target_vel1, 3); Serial.print(",");
        Serial.print(target_vel2, 3); Serial.print(",");
        Serial.println(target_vel3, 3);

    }

    // state machine
    switch (state) {
        case S_MOVE:  do_move();  break;
        case S_WAIT:  do_wait();  break;
        case S_BACK:  do_back();  break;
        case S_CHECK: do_check(); break;
    }
#endif
}