#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

#define Kp     34
#define Kd     0.62
#define Ki     0

#define sampleTime  0.005

const float SetpointAngle = 0.5 ;
float targetAngle = SetpointAngle;

const int left_L1  = 8;
const int left_L2  = 12;
const int PWM_L    = 10;
const int right_R1 = 7;
const int right_R2 = 6;
const int PWM_R    = 9;

MPU6050 mpu;

int16_t accY, accZ;
int16_t gyroX;

float alpha = 0.9934;

volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile float angle_speed=0;
volatile int  count=0;

void Stop() 
{
  digitalWrite(right_R1, LOW);
  digitalWrite(right_R2, LOW);
  digitalWrite(left_L1,  LOW);
  digitalWrite(left_L2,  LOW);
  analogWrite(PWM_R,0);
  analogWrite(PWM_L,0);
}

void Forward(int power) 
{
  digitalWrite(right_R1, HIGH);
  digitalWrite(right_R2, LOW);
  digitalWrite(left_L1,  HIGH);
  digitalWrite(left_L2,  LOW);
  analogWrite(PWM_R, power);
  analogWrite(PWM_L, power);
}

void Backward(int power)
{
  digitalWrite(right_R1, LOW);
  digitalWrite(right_R2, HIGH);
  digitalWrite(left_L1, LOW);
  digitalWrite(left_L2, HIGH);
  analogWrite(PWM_R,power);
  analogWrite(PWM_L,power);
}

void YawRight(int power)
{
  digitalWrite(right_R1, LOW);
  digitalWrite(right_R2, HIGH);
  digitalWrite(left_L1, HIGH);
  digitalWrite(left_L2, LOW);
  analogWrite(PWM_R,power);
  analogWrite(PWM_L,power);
}

void YawLeft(int power)
{
  digitalWrite(right_R1, HIGH);
  digitalWrite(right_R2, LOW);
  digitalWrite(left_L1, LOW);
  digitalWrite(left_L2, HIGH);
  analogWrite(PWM_R,power);
  analogWrite(PWM_L,power);
}

void init_PID()
{  
  // initialize Timer2
  cli();            // disable global interrupts
  TCCR2B = 0;
  TCCR2A = 0;
  OCR2A  = 77;
  
  // Set to CTC Mode
  TCCR2A |= (1 << WGM21);

  // prescaling by 1024
  TCCR2B |= (1 << CS22);
  TCCR2B |= (1 << CS21);
  TCCR2B |= (1 << CS20);
  
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  sei();          // enable global interrupts
}

void setup() 
{
  Serial.begin(9600);

  pinMode(right_R1, OUTPUT);
  pinMode(right_R2, OUTPUT);
  pinMode(PWM_R,    OUTPUT);
  pinMode(left_L1,  OUTPUT);
  pinMode(left_L2,  OUTPUT);
  pinMode(PWM_L,    OUTPUT);
  Stop();
  pinMode(13, OUTPUT);

  // set the accelerometer  to +/- 2g and 
  // the gyroscope to 250% per second by default
  mpu.initialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity  
  mpu.setXAccelOffset(-3071);
  mpu.setYAccelOffset(-269);
  mpu.setZAccelOffset(1387);
  mpu.setXGyroOffset(54);
  mpu.setYGyroOffset(-18);
  mpu.setZGyroOffset(-16);

  init_PID();
}

// The ISR will be called every 5 milliseconds
//ISR(TIMER1_COMPA_vect)
ISR(TIMER2_COMPA_vect)
{
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;
  currentAngle = (alpha*(prevAngle+gyroAngle)) + ((1.0-alpha)*accAngle);
  angle_speed = (currentAngle-prevAngle)/sampleTime;

  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);
  
  //PID control
  motorPower = (Kp*error) + (Ki*errorSum*sampleTime) + (Kd*angle_speed);
  
  prevAngle = currentAngle;

  // toggle the led on pin13 every second
  count++;
  if(count == 200)
  {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}

void loop() {

  // read acceleration and gyroscope values
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();  
  gyroX = mpu.getRotationX();
  
  if(motorPower < 0)
  {
    Forward(-motorPower);
  }
  else
  {
    Backward(motorPower);
  }
}
