#include <Arduino.h>
#include <MsTimer2.h>

/* ================== PIN DEFINITIONS ================== */
// --- Motor 1 ---
#define M1_IN1 22
#define M1_IN2 23
#define M1_PWM 11
#define M1_ENCODER_A 18
#define M1_ENCODER_B 19

// --- Motor 4 ---
#define M4_IN1 24
#define M4_IN2 25
#define M4_PWM 12
#define M4_ENCODER_A 20
#define M4_ENCODER_B 21

/* ================== PID STRUCTURE ================== */
typedef struct {
  float input;
  float output;
  float feedback;
  float k_p;
  float k_i;
  float k_d;
  float err_1;
  float err_2;
  float err_x;
  float out_max;
  float out_min;
  float err_x_max;
} PID;

/* ================== GLOBAL VARIABLES ================== */
volatile float motor_M1 = 0;
volatile float motor_M4 = 0;

int timecnt = 0;
int time_seconds = 0;

PID M1_Motor_PID;
PID M4_Motor_PID;

/* ================== FUNCTION PROTOTYPES ================== */
void Motor_Init(void);
void PID_Init(void);
void PID_Cal(PID *pid);
void Read_Motor_M1(void);
void Read_Motor_M4(void);
void Read_Motor_V(void);
void PID_Cal_Computer_Out(void);
void Motor_Test(void);
void Motor_PWM_Set(float m1_pwm, float m4_pwm);

/* ================== SETUP ================== */
void setup() {
  Motor_Init();
  PID_Init();
  MsTimer2::set(50, PID_Cal_Computer_Out);
  MsTimer2::start();
  Serial.begin(115200);
  Serial.println("Mega2560 Two Motor (M1 & M4) PID Ready");
}

/* ================== MAIN LOOP ================== */
void loop() {
  Motor_Test();
  Read_Motor_V();

  // Print format: Target, M1_Speed, M4_Speed, M1_PWM, M4_PWM
  Serial.print(M1_Motor_PID.input);
  Serial.print(",");
  Serial.print(M1_Motor_PID.feedback);
  Serial.print(",");
  Serial.print(M4_Motor_PID.feedback);
  Serial.print(",");
  Serial.print(M1_Motor_PID.output);
  Serial.print(",");
  Serial.println(M4_Motor_PID.output);
}

/* ================== TEST SEQUENCE ================== */
// TUNE HERE: CHANGE THESE VALUES TO TEST DIFFERENT TARGET SPEEDS (mm/s)
void Motor_Test(void) {
  float target = 0;
  if (time_seconds < 2)
    target = 0; // Stop for 2 seconds
  else if (time_seconds < 4)
    target = 50; // Slow forward
  else if (time_seconds < 6)
    target = 100; // Fast forward
  else if (time_seconds < 8)
    target = -50; // Slow reverse
  else if (time_seconds < 10)
    target = -100; // Fast reverse
  else
    time_seconds = 0;

  M1_Motor_PID.input = target;
  M4_Motor_PID.input = target;
}

/* ================== 50ms PID + PWM OUTPUT ================== */
void PID_Cal_Computer_Out(void) {
  PID_Cal(&M1_Motor_PID);
  PID_Cal(&M4_Motor_PID);

  Motor_PWM_Set(M1_Motor_PID.output, M4_Motor_PID.output);

  timecnt++;
  if (timecnt >= 20) {
    time_seconds++;
    timecnt = 0;
  }
}

/* ================== MOTOR PWM + DIRECTION ================== */
void Motor_PWM_Set(float m1_pwm, float m4_pwm) {
  // Motor 1
  if (m1_pwm > 0) {
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, HIGH);
    analogWrite(M1_PWM, (int)m1_pwm);
  } else {
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, LOW);
    analogWrite(M1_PWM, (int)(-m1_pwm));
  }

  // Motor 4
  if (m4_pwm > 0) {
    digitalWrite(M4_IN1, LOW);
    digitalWrite(M4_IN2, HIGH);
    analogWrite(M4_PWM, (int)m4_pwm);
  } else {
    digitalWrite(M4_IN1, HIGH);
    digitalWrite(M4_IN2, LOW);
    analogWrite(M4_PWM, (int)(-m4_pwm));
  }
}

/* ================== PIN INITIALIZATION ================== */
void Motor_Init(void) {
  pinMode(M1_ENCODER_A, INPUT);
  pinMode(M1_ENCODER_B, INPUT);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);

  pinMode(M4_ENCODER_A, INPUT);
  pinMode(M4_ENCODER_B, INPUT);
  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);
  pinMode(M4_PWM, OUTPUT);

  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  analogWrite(M1_PWM, 0);

  digitalWrite(M4_IN1, LOW);
  digitalWrite(M4_IN2, LOW);
  analogWrite(M4_PWM, 0);
}

/* ================== PID PARAMETERS ================== */
// TUNE HERE: ADJUST THESE VALUES IF THE MOTOR STUTTERS OR RESPONDS TOO SLOWLY
void PID_Init(void) {
  // --- Motor 1 Tuning ---
  M1_Motor_PID.k_p = 0.5; // Proportional: Increase for a stronger/faster kick
                          // towards the target
  M1_Motor_PID.k_i = 0.8; // Integral: Increase to reach exact target speed (too
                          // high = shaking/oscillations)
  M1_Motor_PID.k_d =
      0.1; // Derivative: Increase to dampen/smooth out sudden speed jerks
  M1_Motor_PID.out_max = 250;  // Max forward power limit (Arduino max is 255)
  M1_Motor_PID.out_min = -250; // Max reverse power limit
  M1_Motor_PID.input = 0;
  M1_Motor_PID.err_x_max = 1000; // Integral windup limit (prevents the math
                                 // from building up to infinity)

  // --- Motor 4 Tuning ---
  M4_Motor_PID.k_p = 0.5;
  M4_Motor_PID.k_i = 0.8;
  M4_Motor_PID.k_d = 0.1;
  M4_Motor_PID.out_max = 250;
  M4_Motor_PID.out_min = -250;
  M4_Motor_PID.input = 0;
  M4_Motor_PID.err_x_max = 1000;
}

/* ================== INCREMENTAL PID ================== */
void PID_Cal(PID *pid) {
  pid->err_2 = pid->err_1;
  pid->err_1 = pid->input - pid->feedback;

  float p = pid->k_p * pid->err_1;
  float i = pid->k_i * pid->err_x;
  float d = pid->k_d * (pid->err_1 - pid->err_2);

  pid->err_x += pid->err_1;
  pid->output = p + i + d;

  if (pid->output > pid->out_max)
    pid->output = pid->out_max;
  if (pid->output < pid->out_min)
    pid->output = pid->out_min;

  if (pid->err_x > pid->err_x_max)
    pid->err_x = pid->err_x_max;
  else if (pid->err_x < -pid->err_x_max)
    pid->err_x = -pid->err_x_max;
}

/* ================== ENCODER INTERRUPTS ================== */
// TUNE HERE: FIX "RUNAWAY" POSITIVE FEEDBACK MOTORS
void Read_Motor_M1(void) {
  int dir = digitalRead(M1_ENCODER_B);
  // If Motor 1 spins out of control at full speed, swap the -1 and 1 below!
  motor_M1 += (dir == 1) ? -1 : 1;
}

void Read_Motor_M4(void) {
  int dir = digitalRead(M4_ENCODER_B);
  // If Motor 4 spins out of control at full speed, swap the -1 and 1 below!
  motor_M4 += (dir == 1) ? -1 : 1;
}

/* ================== SPEED CALCULATION ================== */
void Read_Motor_V(void) {
  static float prev_speed_M1 = 0;
  static float prev_speed_M4 = 0;
  const float filter = 0.3;

  unsigned long end_time = millis() + 50;
  motor_M1 = 0;
  motor_M4 = 0;

  attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A), Read_Motor_M1, FALLING);
  attachInterrupt(digitalPinToInterrupt(M4_ENCODER_A), Read_Motor_M4, FALLING);

  while (millis() < end_time)
    ;

  detachInterrupt(digitalPinToInterrupt(M1_ENCODER_A));
  detachInterrupt(digitalPinToInterrupt(M4_ENCODER_A));

  // TUNE HERE: UPDATE THESE NUMBERS IF YOU CHANGE MOTORS OR WHEELS
  // 330.0 = Encoder Pulses Per Revolution
  // 48.0 = Gearbox Ratio
  // 20.0 = Time multiplier (Because we read for 50ms, 1000ms/50ms = 20)
  // 3.14159 = Pi (To calculate wheel circumference)
  float new_speed_M1 = (motor_M1 / 330.0) * 48.0 * 3.14159 * 20.0;
  float new_speed_M4 = (motor_M4 / 330.0) * 48.0 * 3.14159 * 20.0;

  M1_Motor_PID.feedback = (1 - filter) * new_speed_M1 + filter * prev_speed_M1;
  M4_Motor_PID.feedback = (1 - filter) * new_speed_M4 + filter * prev_speed_M4;

  prev_speed_M1 = M1_Motor_PID.feedback;
  prev_speed_M4 = M4_Motor_PID.feedback;
}
