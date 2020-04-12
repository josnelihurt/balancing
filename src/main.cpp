
#include "I2Cdev.h"
#include "MPU9150.h"
#include "helper_3dmath.h"
#include <Arduino.h>
#include <HCSR04.h>
#include <LiquidCrystal_I2C.h>
#include <TM1637Display.h>
#define TRIG_PIN A2
#define ECHO_PIN A3

#define Kp 20
#define Kd 0.02
// kd 0.05
#define Ki 3
#define sampleTime 0.005
#define targetAngle -2

volatile int32_t motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, error,
                                                  prevError = 0, errorSum = 0;

int16_t accY, accZ, gyroX;
MPU9150 accelGyroMag;

UltraSonicDistanceSensor distanceSensor(
    TRIG_PIN,
    ECHO_PIN); // Initialize sensor that uses digital pins 13 and 12.

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

#define pinPWMA 6
#define pinAIN2 7
#define pinAIN1 8
#define pinBIN1 9
#define pinBIN2 10
#define pinPWMB 11
#define pinSTBY 12

#define TEST_DELAY 100
int restict(int amt, int low, int high) {
  return (amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt));
}
void printff(const char *fmt, ...);
void clearTerminal() { Serial.write(12); }
void setupLcd() {
  lcd.begin(16, 2);
  lcd.home();
}

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {

  leftMotorSpeed = restict(leftMotorSpeed, -255, 255);
  rightMotorSpeed = restict(rightMotorSpeed, -255, 255);

  if (leftMotorSpeed >= 0) {
    analogWrite(pinPWMA, leftMotorSpeed);
    digitalWrite(pinAIN1, HIGH);
    digitalWrite(pinAIN2, LOW);
  } else {
    analogWrite(pinPWMA, -256 - leftMotorSpeed);
    digitalWrite(pinAIN1, LOW);
    digitalWrite(pinAIN2, HIGH);
  }

  if (rightMotorSpeed >= 0) {
    analogWrite(pinPWMB, rightMotorSpeed);
    digitalWrite(pinBIN1, HIGH);
    digitalWrite(pinBIN2, LOW);
  } else {
    analogWrite(pinPWMB, -256 - rightMotorSpeed);
    digitalWrite(pinBIN1, LOW);
    digitalWrite(pinBIN2, HIGH);
  }
}
void init_PID() {
  // initialize Timer1
  cli();      // disable global interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  // set compare match register to set sample time 5ms
  OCR1A = 9999;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // enable global interrupts
}
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 5; ++i) {
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on
    delay(50);                       // wait for half a second
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off
    delay(50);                       // wait for half a second
  }
  Serial.begin(115200);
  clearTerminal();
  Wire.begin();
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelGyroMag.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful"
                                               : "MPU9150 connection failed");
  // set the motor control and PWM pins to output mode
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinAIN2, OUTPUT);
  pinMode(pinPWMB, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  digitalWrite(pinSTBY, HIGH);
  setupLcd();
  init_PID();

  //   mpu.initialize();
  //   mpu.setYAccelOffset(1593);
  //   mpu.setZAccelOffset(963);
  //   mpu.setXGyroOffset(40);
  printff("S-ROBOT");
}
void printff(const char *fmt, ...) {
  char tmp[1024];
  va_list args;
  va_start(args, fmt);
  vsnprintf(tmp, 1024, fmt, args);
  va_end(args);

  lcd.print(tmp);
  Serial.print(tmp);
}
void readAngle() {
  accY = accelGyroMag.getAccelerationY();
  accZ = accelGyroMag.getAccelerationZ();
  gyroX = accelGyroMag.getRotationX();
}

int ml = 0;
int mr = 0;
void updateFromSerial() {
  lcd.setCursor(0, 1);

  if (Serial.available() > 0) {
    int incomingByte = Serial.read();
    switch (incomingByte) {
    case '1':
      ml++;
      break;
    case '2':
      ml--;
      break;
    case '3':
      mr++;
      break;
    case '4':
      mr--;
      break;

    default:
      ml = 0;
      mr = 0;
      break;
    }
  }

  printff("\rl:%03d,r:%03d", ml, mr);
  setMotors(ml, mr);
}

void loop() {
  readAngle();
  lcd.setCursor(0, 1);
  int value = restict(motorPower, -255, 255);
  setMotors(value, value);
  //printff("\rM={%4d    }", value);
  if (currentAngle > 50) {
    digitalWrite(pinSTBY, LOW);
  } else if (currentAngle < -50) {
    digitalWrite(pinSTBY, LOW);
  } else {
    digitalWrite(pinSTBY, HIGH);
  }

  // distanceSensor.measureDistanceCm()
}

// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect) {
  static int count = 0;
  // calculate the angle of inclination
  accAngle = atan2(accY, accZ) * RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate * sampleTime;
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * (accAngle);

  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = restict(errorSum, -300, 300);
  // calculate output from P, I and D values
  motorPower = Kp * (error) + Ki * (errorSum)*sampleTime -
               Kd * (currentAngle - prevAngle) / sampleTime;

  prevAngle = currentAngle;
  // toggle the led on pin13 every second
  count++;
  if (count == 200) {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}