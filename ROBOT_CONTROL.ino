#include <WiFi.h>
// เพิ่ม volatile เพื่อความปลอดภัยของข้อมูล
volatile int Counter = 0; 
hw_timer_t * timer = NULL;  
volatile bool rmtloop = false;    

#define Upper1  4
#define Lower1  16
#define ENC1_A 32
#define ENC1_B 33

#define Upper2  17
#define Lower2  18
#define ENC2_A 25
#define ENC2_B 26

#define Upper3  19
#define Lower3  23
#define ENC3_A 34
#define ENC3_B 35

// --- Motor 1 (Front) ---
// ต้องใส่ volatile ทุกตัวที่ onTimer มีการอ่านหรือเขียนค่า
volatile float Kp1 = 1.50;
volatile float Kp2 = 1.50; 
volatile float Kp3 = 1.50;

volatile int pos1 = 0, target1 = 0;
volatile int pos2 = 0, target2 = 0;
volatile int pos3 = 0, target3 = 0;

volatile int controlSignal1 = 0;
volatile int controlSignal2 = 0;
volatile int controlSignal3 = 0;


volatile float PosREAL1 = 0 ;
volatile float PosREAL2 = 0 ;
volatile float PosREAL3 = 0 ;

// ตัวแปรที่ใช้คำนวณภายใน onTimer เท่านั้น ไม่ต้อง volatile ก็ได้ 
// แต่แนะนำให้ประกาศเป็น volatile ไว้ก่อนถ้าจะเอาไป Print ดูใน Loop ครับ
volatile float err1 = 0, err2 = 0, err3 = 0;
volatile int pwm1 = 0, pwm2 = 0, pwm3 = 0;
volatile float Ki1 = 0.01;
volatile float Ki2 = 0.01;
volatile float Ki3 = 0.01; // เริ่มจากค่าน้อยๆ ก่อนเสมอ (เช่น 0.01 - 0.05)
volatile float sumErr1 = 0;
volatile float sumErr2 = 0;
volatile float sumErr3 = 0;
volatile int maxIntegral = 100;


// PID Variables
float errorValue = 0; 

int controlSignal = 0;
float targetPosition = 0;
volatile float velocurren = 0;
volatile int currentPos = 0;
volatile float Countelast = 0;
const float dt = 0.01; // แก้จาก int เป็น float
int Volte = 12;//แรงไฟ
bool smp = false;
int PWMvalue1 = 0;
float target =0;
volatile int motorDirection = 0;
float targetX = 0;
float targetY = 0;


// --- ข้อมูลล้อและ Encoder ---
const float WHEEL_DIAMETER = 0.082; // เมตร
const float PPR = 4.0;            // สมมติว่า 11 (ลองเช็คสเปคมอเตอร์อีกที)
const float GEAR_RATIO = 19.0;      // สมมติว่า 30 (ลองเช็คสเปคมอเตอร์อีกที)
const int DECODING_MODE = 4;        // เราใช้ 4x
const float PI_VALUE = 3.14159265;
// 1 รอบล้อจะได้กี่ Tick?
const float TICKS_PER_REV = PPR*GEAR_RATIO*DECODING_MODE; 
// 1 เมตรจะได้กี่ Tick?
const float TICKS_PER_METER = TICKS_PER_REV / (WHEEL_DIAMETER * PI_VALUE);
// Interrupt Service Routine สำหรับ Encoder
const float METER_PER_TICKS = 0.000847;
volatile int step = 0;               // 0: idle, 1: go, 2: pause, 3: back
volatile unsigned long pauseTimer = 0;


volatile int countA1 = 0;
volatile int countB1 = 0;
volatile int Counter1 = 0; 


volatile int countA2 = 0;
volatile int countB2 = 0;
volatile int Counter2 = 0; 

volatile int countA3 = 0;
volatile int countB3 = 0;
volatile int Counter3 = 0; 


volatile bool readyToCompute = false;
volatile float cycleCount = 0;        // ตัวนับรอบ
volatile float totalCycles = 65.0; // ต้องจบใน 65 รอบ (0.65 วิ)
volatile float finalTarget = 304.0;  // เป้าหมาย 304 Ticks
bool isMoving = false;
volatile float currentTarget1 = 0 ;
volatile float currentTarget2 = 0 ;
volatile float currentTarget3 = 0;
volatile float finalT1 =0;
volatile float finalT2 =0;
volatile float finalT3 =0;
volatile float progress = 0;
#include <WiFiUdp.h>

// ==========================================
// การตั้งค่า WiFi
// ==========================================
const char* ssid = "MCE14";      
const char* password = "12345678"; 
const int localUdpPort = 12345; 

WiFiUDP udp;

// ตัวแปรสำหรับเช็คข้อมูลที่มาหลงลำดับ (Out-of-order)
uint32_t lastSeqNum = 0;
bool isFirstPacket = true;

// ตัวแปรสำหรับเช็คสถานะ WiFi
unsigned long lastWiFiCheck = 0;

void IRAM_ATTR readEncoderISR1() {
  int a1 = digitalRead(ENC1_A);
  int b1 = digitalRead(ENC1_B);
  static int lastA1 = 0;
  static int lastB1 = 0;

  // --- ส่วน Debug ---
  if (a1 != lastA1) countA1++; 
  if (b1 != lastB1) countB1++; 

  // --- ส่วนคำนวณ Counter (4x Logic) ---
  // เช็คทุกครั้งที่ A เปลี่ยน
  if (a1 != lastA1) {
    if (a1 != b1) Counter1++; else Counter1--;
  }

  // เช็คทุกครั้งที่ B เปลี่ยน (ห้ามใช้ else if!)
  if (b1 != lastB1) {
    if (b1== a1) Counter1++; else Counter1 --;
  }

  lastA1 = a1;
  lastB1 = b1;
}
void IRAM_ATTR readEncoderISR2() {
  int a2 = digitalRead(ENC2_A);
  int b2 = digitalRead(ENC2_B);
  static int lastA2 = 0;
  static int lastB2 = 0;

  // --- ส่วน Debug ---
  if (a2 != lastA2) countA2++; 
  if (b2 != lastB2) countB2++; 

  // --- ส่วนคำนวณ Counter (4x Logic) ---
  // เช็คทุกครั้งที่ A เปลี่ยน
  if (a2 != lastA2) {
    if (a2 != b2) Counter2++; else Counter2--;
  }

  // เช็คทุกครั้งที่ B เปลี่ยน (ห้ามใช้ else if!)
  if (b2 != lastB2) {
    if (b2 == a2) Counter2++; else Counter2--;
  }

  lastA2 = a2;
  lastB2 = b2;
}
void IRAM_ATTR readEncoderISR3() {
  int a3 = digitalRead(ENC3_A);
  int b3 = digitalRead(ENC3_B);
  static int lastA3 = 0;
  static int lastB3 = 0;

  // --- ส่วน Debug ---
  if (a3 != lastA3) countA3++; 
  if (b3 != lastB3) countB3++; 

  // --- ส่วนคำนวณ Counter (4x Logic) ---
  // เช็คทุกครั้งที่ A เปลี่ยน
  if (a3 != lastA3) {
    if (a3 != b3) Counter3++; else Counter3--;
  }

  // เช็คทุกครั้งที่ B เปลี่ยน (ห้ามใช้ else if!)
  if (b3 != lastB3) {
    if (b3 == a3) Counter3++; else Counter3--;
  }

  lastA3 = a3;
  lastB3 = b3;
}

void IRAM_ATTR onTimer() {
  readyToCompute = true; // แค่ยกธงบอกว่าถึงเวลา 10ms แล้วนะ
}


void setupWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  // ป้องกันการค้างลูปถาวร กรณีเราเตอร์ล่ม ให้รอแค่ 10 วินาทีต่อรอบ
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP()); 
    udp.begin(localUdpPort);
  } else {
    Serial.println("\nWiFi Failed to connect. Will retry later.");
  }
}
TaskHandle_t MotorTask;

void MotorControlLoop(void * pvParameters) {
  for(;;) {
    if (readyToCompute) { 
        readyToCompute = false;

        if (isMoving) {
            cycleCount++; 

            if (cycleCount <= totalCycles) {
                // 1. คำนวณ progress ตามปกติ (อย่าลืมใส่ (float) เพื่อป้องกันค่าเป็น 0)
                progress = cycleCount / totalCycles;

                // --- ส่วนที่ปรับปรุง: การจัดการทิศทางตาม Step ---
                float currentTarget1, currentTarget2, currentTarget3;
                
                if (step == 1) { // ขาไป: 0.0 -> 1.0 (0 ถึง 304 Ticks)
                    currentTarget1 = finalT1 * progress;
                    currentTarget2 = finalT2 * progress;
                    currentTarget3 = finalT3 * progress;
                } else if (step == 3) { // ขากลับ: 1.0 -> 0.0 (304 ถึง 0 Ticks)
                    currentTarget1 = finalT1 * (1.0 - progress);
                    currentTarget2 = finalT2 * (1.0 - progress);
                    currentTarget3 = finalT3 * (1.0 - progress);
                }

                // 2. คำนวณ PID (ใช้โค้ดเดิมของคุณได้เลย)
                err1 = currentTarget1 - Counter1;
                sumErr1 = constrain(sumErr1 + err1, -maxIntegral, maxIntegral);
                pwm1 = (Kp1 * err1) + (Ki1 * sumErr1);

                err2 = currentTarget2 - Counter2;
                sumErr2 = constrain(sumErr2 + err2, -maxIntegral, maxIntegral);
                pwm2 = (Kp2 * err2) + (Ki2 * sumErr2);  

                err3 = currentTarget3 - Counter3;
                sumErr3 = constrain(sumErr3 + err3, -maxIntegral, maxIntegral);
                pwm3 = (Kp3 * err3) + (Ki3 * sumErr3); 

            } else {
                // เมื่อครบ 65 รอบ
                isMoving = false; 
                pwm1 = 0; pwm2 = 0; pwm3 = 0;
                sumErr1 = 0; sumErr2 = 0; sumErr3 = 0; // เคลียร์ Integral ทุกครั้งที่จบช่วง

                if (step == 1) {
                    step = 2; // จบขาไป ให้ไปสถานะ "พัก"
                    pauseTimer = millis(); 
                } else if (step == 3) {
                    step = 0; // จบขากลับ ให้หยุดทำงาน
                    Serial.println("Back to Start point.");
                }
            }

            // 3. สั่งงานมอเตอร์
            driveMotor1(constrain(pwm1, -255, 255));
            driveMotor2(constrain(pwm2, -255, 255));
            driveMotor3(constrain(pwm3, -255, 255));
        }

        // --- ส่วนตรวจสอบการพัก (Delay โดยไม่หยุด CPU) ---
        if (step == 2) {
            if (millis() - pauseTimer > 2000) { // พัก 2 วินาที
                cycleCount = 0; // รีเซ็ตตัวนับเพื่อเริ่มนับ 1-65 ใหม่
                isMoving = true;
                step = 3; // เปลี่ยนเป็นขากลับ
                Serial.println("Returning Home...");
            }
        }
    }
    vTaskDelay(1);
  }
}
void setup() {
  Serial.begin(115200);
  setupWiFi();
  
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  
  pinMode(ENC3_A, INPUT_PULLUP);
  pinMode(ENC3_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), readEncoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), readEncoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), readEncoderISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), readEncoderISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), readEncoderISR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_B), readEncoderISR3, CHANGE);
  
  // ตั้งค่า PWM สำหรับ ESP32
  ledcSetup(0, 5000, 8); 
  ledcSetup(1, 5000, 8); 
  ledcAttachPin(Upper1, 0); 
  ledcAttachPin(Lower1, 1);

  ledcSetup(2, 5000, 8); 
  ledcSetup(3, 5000, 8); 
  ledcAttachPin(Upper2, 2); 
  ledcAttachPin(Lower2, 3);

  ledcSetup(4, 5000, 8); 
  ledcSetup(5, 5000, 8); 
  ledcAttachPin(Upper3, 4); 
  ledcAttachPin(Lower3, 5);
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000, true);
  timerAlarmEnable(timer);

  // 4. สร้าง Task ให้ไปรันที่ Core 0
  xTaskCreatePinnedToCore(
    MotorControlLoop,   /* ฟังก์ชันที่จะรัน */
    "MotorTask",        /* ชื่อ Task */
    10000,              /* Stack size */
    NULL,               /* Parameter */
    1,                  /* Priority */
    &MotorTask,         /* Handle */
    0                   /* Core 0 */
  );
}

void loop() {
  // 1. ระบบรักษาการเชื่อมต่อ (Auto Reconnect)
  // หากหลุด WiFi จะพยายามต่อใหม่ทุก 5 วินาที โดยไม่ทำให้โค้ดส่วนอื่นค้างตลอดไป
  if (millis() - lastWiFiCheck > 5000) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost. Reconnecting...");
      setupWiFi();
    }
    lastWiFiCheck = millis();
  }

  // 2. ตรวจสอบข้อมูล UDP
  int packetSize = udp.parsePacket();
  if (packetSize) {
    
    // คาดหวังแพ็กเกจจนาด 16 Bytes (Sequence 4 + X 4 + Y 4 + Z 4)
    if (packetSize == 16) {
      uint32_t seqNum;
      float x, y, z;
      
      // อ่านข้อมูลทีละ 4 bytes ลงในตัวแปร
      udp.read((char*)&seqNum, 4);
      udp.read((char*)&x, 4);
      udp.read((char*)&y, 4);
      udp.read((char*)&z, 4);
      
      // 3. ป้องกันข้อมูลหลงลำดับ (Out-of-Order Packets Guard)
      // บางครั้ง UDP แพ็กเกจใหม่เดินทางมาถึงก่อนหน้าแพ็กเกจเก่า 
      // ถ้ารับแพ็กเกจเก่าไปใช้ หุ่นยนต์อาจจะกระตุกถอยหลังชั่วขณะ
      if (isFirstPacket || seqNum > lastSeqNum) {
        lastSeqNum = seqNum;
        isFirstPacket = false;
        
        // 1. คำนวณ Kinematics พื้นฐานสำหรับหุ่น Omni 3 ล้อ
        // x, y ที่ได้รับมาจะถูกแปลงเป็นเป้าหมายของแต่ละล้อ
        float tx = Z * TICKS_PER_METER;
        float ty = x * TICKS_PER_METER;

        finalT1 = ty;
        finalT2 = -0.866 * tx - 0.5 * ty;
        finalT3 =  0.866 * tx - 0.5 * ty;

        // 2. เริ่มต้นระบบ Sequence (เฉพาะเมื่อหุ่นว่างงานอยู่)
        if (step == 0) {
            cycleCount = 0;
            step = 1;      // เริ่มขาไป
            isMoving = true;
            Serial.println("Starting Mission...");
        }
        // Print ดูค่า (เอาออกได้เพื่อให้ทำงานไวขึ้นสุดขีด)
        Serial.print("Seq:"); Serial.print(seqNum);
        Serial.print("\tX:"); Serial.print(x, 2);
        Serial.print(" Y:"); Serial.print(y, 2);
        Serial.print(" Z:"); Serial.println(z, 2);
        

        // --- เพิ่มโค้ดควบคุม Motor / Servo ตรงนี้ ---
        
      } else {
        // หากมีแพ็กเกจที่ช้าและเก่ากว่าค่าล่าสุดมาถึง ให้ทิ้งไป
        // Serial.println("Ignored old packet");
      }
      
    } else {
      // ล้างข้อมูลขยะที่ส่งมาผิดขนาดออกจาก Buffer
      udp.flush();
    }
  }
  
  // ระวัง: ห้ามใช้ delay() นานๆ ใน loop เพื่อให้รอบการรันเร็วที่สุด
}
void driveMotor1(int controlSignal1) { // รับค่าจากล้อใครล้อมัน
    int motorDirection = 0;
    
    // 1. จัดการทิศทาง
    if (controlSignal1 > 0) motorDirection = 1;
    else if (controlSignal1 < 0) motorDirection = -1;
    else motorDirection = 0;

    // 2. จัดการความเร็ว (PWM)
    int absPWM = abs(controlSignal1);
    int finalPWM = 0;

    // ระบบ Deadzone Compensation (สำคัญมากสำหรับหุ่น 4kg)
    int deadzone = 45; // ค่า PWM น้อยที่สุดที่ทำให้ล้อเริ่มขยับจริง (ต้องลองจูนดู)

    if (absPWM > 0) {
        // สูตร: เอา PID มาบวกกับ Deadzone เพื่อให้มอเตอร์มีแรงชนะแรงเสียดทานทันที
        finalPWM = absPWM + deadzone; 
    }

    // ล็อคไม่ให้เกิน 255 (ป้องกัน Register พัง)
    if (finalPWM > 255) finalPWM = 255;
    
    // ถ้าเป้าหมายจบแล้ว หรือ Error น้อยมาก ให้หยุดสนิท
    if (abs(err1) <= 1) finalPWM = 0;

    // 3. สั่งงาน LEDC (ล้อ 1 ใช้ Channel 0, 1)
    if (motorDirection == 1) {
        ledcWrite(0, 0);
        ledcWrite(1, finalPWM);
    } else if (motorDirection == -1) {
        ledcWrite(0, finalPWM);
        ledcWrite(1, 0);
    } else {
        // Active Brake: ถ้าอยากให้หยุดกึ๊ก ให้สั่ง HIGH ทั้งคู่ (ถ้า Driver รองรับ)
        // แต่เบื้องต้นสั่ง 0 ทั้งคู่ก่อนก็ได้ครับ
        ledcWrite(0, 0);
        ledcWrite(1, 0);
    }
}
void driveMotor2(int controlSignal2) { // รับค่าจากล้อใครล้อมัน
    int motorDirection = 0;
    
    // 1. จัดการทิศทาง
    if (controlSignal2 > 0) motorDirection = 1;
    else if (controlSignal2 < 0) motorDirection = -1;
    else motorDirection = 0;

    // 2. จัดการความเร็ว (PWM)
     int absPWM = abs(controlSignal2);
    int finalPWM = 0;

    // ระบบ Deadzone Compensation (สำคัญมากสำหรับหุ่น 4kg)
    int deadzone = 45; // ค่า PWM น้อยที่สุดที่ทำให้ล้อเริ่มขยับจริง (ต้องลองจูนดู)

    if (absPWM > 0) {
        // สูตร: เอา PID มาบวกกับ Deadzone เพื่อให้มอเตอร์มีแรงชนะแรงเสียดทานทันที
        finalPWM = absPWM + deadzone; 
    }

    // ล็อคไม่ให้เกิน 255 (ป้องกัน Register พัง)
    if (finalPWM > 255) finalPWM = 255;
    
    // ถ้าเป้าหมายจบแล้ว หรือ Error น้อยมาก ให้หยุดสนิท
    if (abs(err2) <= 1) finalPWM = 0;
    // 3. สั่งงาน LEDC (ล้อ 1 ใช้ Channel 0, 1)
    
    if (motorDirection == 1) {
        ledcWrite(2, 0);
        ledcWrite(3, finalPWM);
    } else if (motorDirection == -1) {
        ledcWrite(2, finalPWM);
        ledcWrite(3, 0);
    } else {
        // Active Brake: ถ้าอยากให้หยุดกึ๊ก ให้สั่ง HIGH ทั้งคู่ (ถ้า Driver รองรับ)
        // แต่เบื้องต้นสั่ง 0 ทั้งคู่ก่อนก็ได้ครับ
        ledcWrite(2, 0);
        ledcWrite(3, 0);
    }
}
void driveMotor3(int controlSignal3) { // รับค่าจากล้อใครล้อมัน
    int motorDirection = 0;
    
    // 1. จัดการทิศทาง
    if (controlSignal3 > 0) motorDirection = 1;
    else if (controlSignal3 < 0) motorDirection = -1;
    else motorDirection = 0;

    // 2. จัดการความเร็ว (PWM)
     int absPWM = abs(controlSignal3);
    int finalPWM = 0;

    // ระบบ Deadzone Compensation (สำคัญมากสำหรับหุ่น 4kg)
    int deadzone = 45; // ค่า PWM น้อยที่สุดที่ทำให้ล้อเริ่มขยับจริง (ต้องลองจูนดู)

    if (absPWM > 0) {
        // สูตร: เอา PID มาบวกกับ Deadzone เพื่อให้มอเตอร์มีแรงชนะแรงเสียดทานทันที
        finalPWM = absPWM + deadzone; 
    }

    // ล็อคไม่ให้เกิน 255 (ป้องกัน Register พัง)
    if (finalPWM > 255) finalPWM = 255;
    
    // ถ้าเป้าหมายจบแล้ว หรือ Error น้อยมาก ให้หยุดสนิท
    if (abs(err3) <= 1) finalPWM = 0;

    // 3. สั่งงาน LEDC (ล้อ 1 ใช้ Channel 0, 1)
    if (motorDirection == 1) {
        ledcWrite(4, 0);
        ledcWrite(5, finalPWM);
    } else if (motorDirection == -1) {
        ledcWrite(4, finalPWM);
        ledcWrite(5, 0);
    } else {
        // Active Brake: ถ้าอยากให้หยุดกึ๊ก ให้สั่ง HIGH ทั้งคู่ (ถ้า Driver รองรับ)
        // แต่เบื้องต้นสั่ง 0 ทั้งคู่ก่อนก็ได้ครับ
        ledcWrite(4, 0);
        ledcWrite(5, 0);
    }
   
}