#include <Wire.h>
#include <MPU6050.h>

// Pin Definitions
#define MOTOR_LEFT_PWM 25    // PWM control for left motor
#define MOTOR_RIGHT_PWM 26   // PWM control for right motor
#define MOTOR_LEFT_DIR 27    // Direction control left motor
#define MOTOR_RIGHT_DIR 14   // Direction control right motor
#define MPU_SDA 21
#define MPU_SCL 22

// PWM Configuration
#define PWM_FREQ 5000        // 5KHz
#define PWM_RES 8           // 8-bit resolution
#define PWM_L_CH 0          // PWM channel 0
#define PWM_R_CH 1          // PWM channel 1

// PID Constants
#define Kp 15.0
#define Ki 1.2
#define Kd 0.8

MPU6050 mpu;

float angle = 0;
float angleSpeed = 0;
float targetAngle = 0;
float lastError = 0;
float integral = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(MPU_SDA, MPU_SCL);
  
  // Configure motor control pins
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  
  // Configure PWM
  ledcSetup(PWM_L_CH, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_R_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_LEFT_PWM, PWM_L_CH);
  ledcAttachPin(MOTOR_RIGHT_PWM, PWM_R_CH);
  
  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
}

void setMotorSpeed(int16_t leftSpeed, int16_t rightSpeed) {
  // Set motor directions
  digitalWrite(MOTOR_LEFT_DIR, leftSpeed >= 0);
  digitalWrite(MOTOR_RIGHT_DIR, rightSpeed >= 0);
  
  // Set PWM values (0-255)
  ledcWrite(PWM_L_CH, abs(leftSpeed));
  ledcWrite(PWM_R_CH, abs(rightSpeed));
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // Read sensor data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Calculate angle using complementary filter
  float accAngle = atan2(ay, az) * 180 / PI;
  angleSpeed = gx / 131.0;
  angle = 0.96 * (angle + angleSpeed * deltaTime) + 0.04 * accAngle;
  
  // PID calculation
  float error = angle - targetAngle;
  integral = constrain(integral + error * deltaTime, -255, 255);
  float derivative = (error - lastError) / deltaTime;
  lastError = error;
  
  // Calculate motor output using PID
  float output = Kp * error + Ki * integral + Kd * derivative;
  
  // Convert PID output to motor speeds (-255 to 255)
  int16_t leftSpeed = constrain(output, -255, 255);
  int16_t rightSpeed = constrain(-output, -255, 255);
  
  setMotorSpeed(leftSpeed, rightSpeed);
  
  delay(10);
}