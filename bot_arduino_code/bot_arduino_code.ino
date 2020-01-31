/*
*
* Project Name:   Automa
* Author List:    Soofiyan Atar
* Filename:     Arduino code for the bot
* Functions:    mpu_values(), motor_run1(int speed), motor_run2(int speed), mpu_setup(), motor_setup(), 
*               encoder_setup(), timer_int_setup(), lqr_run()
* Global Variables:  uint8_t i, int error = 0, byte type = 0, byte vibrate = 0, bool timer_lqr_run, 
*                    bool blinkState = false, bool dmpReady = false, uint8_t mpuIntStatus, 
*                    uint8_t devStatus, uint16_t packetSize, uint16_t fifoCount, uint8_t fifoBuffer[64], 
*                    int pitch_mpu = 0, int roll_mpu = 0, int yaw_mpu = 0, Quaternion q, VectorInt16 aa, 
*                    VectorInt16 aaReal, VectorInt16 aaWorld, VectorFloat gravity, float euler[3], float ypr[3], uint8_t teapotPacket[14],
*
*/
#include <PinChangeInt.h>

#define motor1_dir1 4
#define motor1_dir2 5
#define motor2_dir1 7
#define motor2_dir2 2//changed 2 to 0
#define motor3_dir1 9
#define motor3_dir2 8
#define motor4_dir1 12
#define motor4_dir2 13
#define motor1_pwm 3
#define motor2_pwm 6
#define motor3_pwm 10
#define motor4_pwm 11

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL


bool timer_lqr_run = 0;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

uint8_t i;

int error = 0;
byte type = 0;
byte vibrate = 0;

bool blinkState = false;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
int pitch_mpu = 0;
int roll_mpu = 0;
int yaw_mpu = 0;

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}


volatile int lastEncoded = 0;
volatile long encoderValue = 0;

volatile int lastEncoded_pcint = 0;
volatile long encoderValue_pcint = 0;

int encoder2A = A0;
int encoder2B = A1;

int encoder1A = A2;
int encoder1B = A3;

float K[2][6] = { {0.7, 7.6, 2320.5, 147.8, -0.7, -0.7}, {0.7, 7.6, 2320.5, 147.8, 0.7, 0.7}};
float u[6][1] = {{0}, {0}, {0}, {0}, {0}, {0}};

float tot;
float torque[2][1];

int pitch_error,yaw_error,x_error;
int pitch_error_vel,yaw_error_vel,x_error_vel;
int pitch_targ,yaw_targ,x_targ;
int pitch_targ_vel,yaw_targ_vel,x_targ_vel;
int prev_pitch=180,prev_yaw=180,prev_x=0;
double x;

int timer1_counter;

void mpu_values();
void motor_run1(int speed);
void motor_run2(int speed);
void mpu_setup();
void motor_setup();
void encoder_setup();
void timer_int_setup();
void lqr_run();

void setup() {
  mpu_setup();
  motor_setup();
  encoder_setup();
  timer_int_setup();
  motor_run1(0);
  motor_run2(0);
  delay(5000);
}

/*
*
* Function Name:   loop
* Input:  None
* Output:  lqr implementation of the bot for self balacning
* Logic:   given some pre determined values to get the bot at that particular parameters
* Example Call: None
*
*/
void loop() {
//  Serial.println(encoderValue);
//  Serial.println(encoderValue_pcint);
  int encoderValue1 = encoderValue/4;
  int encoderValue_pcint1 = encoderValue_pcint/4;
  x = sqrt(encoderValue1*encoderValue1 + encoderValue_pcint1*encoderValue_pcint1);
  mpu_values();
  pitch_targ = 0;
  yaw_targ = 0;
  x_targ = 0;
  for(int i=0;i<3000;i++)
  {
    lqr_run();
    delay(1);
  }
  pitch_targ = 10*(3.14/180);
  yaw_targ = 0;
  x_targ = 1.5;
  for(int i=0;i<4000;i++)
  {
    lqr_run();
    delay(1);
  }
  pitch_targ = 0*(3.14/180);
  yaw_targ = 3.14;
  x_targ = 0;
  for(int i=0;i<4000;i++)
  {
    lqr_run();
    delay(1);
  }
  pitch_targ = 10*(3.14/180);
  yaw_targ = 0;
  x_targ = -1.5;
  for(int i=0;i<4000;i++)
  {
    lqr_run();
    delay(1);
  }
  pitch_targ = 0*(3.14/180);
  yaw_targ = -3.14;
  x_targ = 0;
  for(int i=0;i<4000;i++)
  {
    lqr_run();
    delay(1);
  }
  pitch_targ = 0;
  yaw_targ = 0;
  x_targ = 0;
  for(int i=0;i<3000;i++)
  {
    lqr_run();
    delay(1);
  }
  pitch_targ = 10*(3.14/180);
  yaw_targ = 0;
  x_targ = -2;
  for(int i=0;i<3000;i++)
  {
    lqr_run();
    delay(1);
  }
  pitch_targ = 0;
  yaw_targ = 0;
  x_targ = 0;
  for(int i=0;i<3000;i++)
  {
    lqr_run();
    delay(1);
  }
}

/*
*
* Function Name:   lqr_run
* Input:  none
* Output:     output values to the motor
* Logic:    matrix multiplication of K and u and then feeding to the motor pwm values
* Example Call:   lqr_run()
*
*/
void lqr_run()
{
  if(timer_lqr_run == 1)
  {
    u[0][0] = x_error;
    u[1][0] = x_error_vel;
    u[2][0] = pitch_error;
    u[3][0] = pitch_error_vel;
    u[4][0] = yaw_error;
    u[5][0] = yaw_error_vel;
    for (int i = 0; i < 2; i++)
    {
      for (int j = 0; j < 1; j++)
      {
        for (int k = 0; k < 6; k++)
        {
          tot = tot + K[i][k] * u[k][j];
        }
        torque[i][j] = -tot;
      }
    }
    motor_run1(int(torque[0][0]));
    motor_run2(int(torque[1][0])); 
    timer_lqr_run = 0;
  } 
}

/*
*
* Function Name:   encoder_setup
* Input:  none
* Output:  setting up the encoder for input interrrupt pcint
* Logic:    none
* Example Call:   encoder_setup()
*
*/
void encoder_setup()
{
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);

  digitalWrite(encoder1A, HIGH); //turn pullup resistor on
  digitalWrite(encoder1B, HIGH);  //turn pullup resistor on

  pinMode(encoder2A, INPUT);
  pinMode(encoder2B, INPUT);

  digitalWrite(encoder2A, HIGH);
  digitalWrite(encoder2B, HIGH);

  attachPinChangeInterrupt(encoder2A, updateEncoder, CHANGE);
  attachPinChangeInterrupt(encoder2B, updateEncoder, CHANGE);
  attachPinChangeInterrupt(encoder1A, updateEncoder_pcint, CHANGE);
  attachPinChangeInterrupt(encoder1B, updateEncoder_pcint, CHANGE);
}

/*
*
* Function Name:  motor_setup
* Input:  none
* Output:  setting up the motor for output
* Logic:    none
* Example Call:   motor_setup()
*
*/
void motor_setup()
{
  pinMode(motor1_dir1, OUTPUT);
  pinMode(motor2_dir1, OUTPUT);
  pinMode(motor3_dir1, OUTPUT);
  pinMode(motor4_dir1, OUTPUT);
  pinMode(motor1_dir2, OUTPUT);
  pinMode(motor2_dir2, OUTPUT);
  pinMode(motor3_dir2, OUTPUT);
  pinMode(motor4_dir2, OUTPUT);
  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);
  pinMode(motor3_pwm, OUTPUT);
  pinMode(motor4_pwm, OUTPUT);
}

/*
*
* Function Name:   timer_int_setup
* Input:  none
* Output:  setting up the timer for input interrrupt overflow
* Logic:    none
* Example Call:   timer_int_setup()
*
*/
void timer_int_setup()
{
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  //timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  timer1_counter = 53036;   // preload timer 65536-16MHz/256/5Hz
  
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

/*
*
* Function Name:   mpu_setup
* Input:  none
* Output:  setting up the mpu for I2C
* Logic:    none
* Example Call:  mpu_setup()
*
*/
void mpu_setup()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24;
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
    Serial.begin(9600);
    while (!Serial);
    mpu.initialize();
  
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(65);//65
    mpu.setYGyroOffset(-16);//-16
    mpu.setZGyroOffset(-14);//-14
    mpu.setZAccelOffset(1368); // 1368 factory default for my test chip    // 1196
  
    if (devStatus == 0)
    {
      mpu.setDMPEnabled(true);
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
  
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
    }
}

/*
*
* Function Name:   updateEncoder
* Input:  none
* Output:  ISR for encoder interrupt
* Logic:   reading pin values of the both the pins and then determining encoder values
* Example Call:   updateEncoder()
*
*/
void updateEncoder() {
  int MSB = digitalRead(encoder2A); //MSB = most significant bit
  int LSB = digitalRead(encoder2B); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //store this value for next time
}

/*
*
* Function Name:   updateEncoder_pcint
* Input:  none
* Output:  ISR for encoder interrupt
* Logic:   reading pin values of the both the pins and then determining encoder values
* Example Call:   updateEncoder_pcint()
*
*/
void updateEncoder_pcint() {
  int MSB = digitalRead(encoder1A); //MSB = most significant bit
  int LSB = digitalRead(encoder1B); //LSB = least significant bit

  int encoded_pcint = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded_pcint << 2) | encoded_pcint; //adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_pcint ++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_pcint --;

  lastEncoded_pcint = encoded_pcint; //store this value for next time
}

/*
*
* Function Name:   motor_run1
* Input:  integer values (speed of the motor)
* Output:  motor running at a particular pwm
* Logic:   if positive value run motor in CW else in CCW
* Example Call:   motor_run1(100)
*
*/
void motor_run1(int speed)
{
  //  speed = limit_var(speed,-255,255);
  if (speed > 45)
  {
    digitalWrite(motor1_dir1, HIGH);
    digitalWrite(motor1_dir2, LOW);
    analogWrite(motor1_pwm, speed);
    digitalWrite(motor2_dir1, HIGH);
    digitalWrite(motor2_dir2, LOW);
    analogWrite(motor2_pwm, speed);
  }
  else if (speed < -45)
  {
    speed = -speed;
    digitalWrite(motor1_dir1, LOW);
    digitalWrite(motor1_dir2, HIGH);
    analogWrite(motor1_pwm, speed);
    digitalWrite(motor2_dir1, LOW);
    digitalWrite(motor2_dir2, HIGH);
    analogWrite(motor2_pwm, speed);
  }
  else
  {
    digitalWrite(motor1_dir1, LOW);
    digitalWrite(motor1_dir2, LOW);
    analogWrite(motor1_pwm, 0);
    digitalWrite(motor2_dir1, LOW);
    digitalWrite(motor2_dir2, LOW);
    analogWrite(motor2_pwm, 0);
  }
}

/*
*
* Function Name:   motor_run2
* Input:  integer values (speed of the motor)
* Output:  motor running at a particular pwm
* Logic:   if positive value run motor in CW else in CCW
* Example Call:   motor_run2(100)
*
*/
void motor_run2(int speed)
{
  //  speed = limit_var(speed,-255,255);
  if (speed > 45)
  {
    digitalWrite(motor3_dir1, HIGH);
    digitalWrite(motor3_dir2, LOW);
    analogWrite(motor3_pwm, speed);
    digitalWrite(motor4_dir1, LOW);
    digitalWrite(motor4_dir2, HIGH);
    analogWrite(motor4_pwm, speed);
  }
  else if (speed < -45)
  {
    speed = -speed;
    digitalWrite(motor3_dir1, LOW);
    digitalWrite(motor3_dir2, HIGH);
    analogWrite(motor3_pwm, speed);
    digitalWrite(motor4_dir1, HIGH);
    digitalWrite(motor4_dir2, LOW);
    analogWrite(motor4_pwm, speed);
  }
  else
  {
    digitalWrite(motor3_dir1, LOW);
    digitalWrite(motor3_dir2, LOW);
    analogWrite(motor3_pwm, 0);
    digitalWrite(motor4_dir1, LOW);
    digitalWrite(motor4_dir2, LOW);
    analogWrite(motor4_pwm, 0);
  }
}

/*
*
* Function Name:   mpu_values
* Input:  none
* Output:  Yaw, pitch and roll values
* Logic:   This function is used to get the yaw, pitch and roll values using mathematics of quaternions.
* Example Call:   mpu_values()
*
*/
void mpu_values()
{
  if (!dmpReady) return;
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
  }
  else if (mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    char x = 209, y = 210, z = 211;
    int bot_yaw = map((ypr[0] * 180 / M_PI), -180, 180, 0, 360);
    int bot_pitch = map((ypr[1] * 180 / M_PI), -180, 180, 0, 360);
    int bot_roll = map((ypr[2] * 180 / M_PI), -180, 180, 0, 360);

#endif
    pitch_mpu = bot_pitch;
    roll_mpu = bot_roll;
    yaw_mpu = bot_yaw;
  }
}

/*
*
* Function Name:   update_Encoder
* Input:  interrupt vector
* Output:  ISR for timer overflow interrupt
* Logic:   getting interrupt at every 200 ms
* Example Call:   ISR(TIMER1_OVF_vect)
*
*/
ISR(TIMER1_OVF_vect)        // interrupt service routine 
{
  TCNT1 = timer1_counter;   // preload timer
  pitch_error = pitch_mpu - pitch_targ;
  pitch_error_vel = (pitch_mpu - prev_pitch)/0.2 - pitch_targ_vel;
  yaw_error = yaw_mpu - yaw_targ;
  yaw_error_vel = (yaw_mpu - prev_yaw)/0.2 - yaw_targ_vel;
  x_error = x_targ - int(x);
  x_error_vel = (int(x) - prev_x)/0.2 - x_targ_vel;
  prev_pitch = pitch_mpu;
  prev_yaw = yaw_mpu;
  prev_x = int(x);
  timer_lqr_run = 1;
}
