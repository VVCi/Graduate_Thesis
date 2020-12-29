#include "Simple_MPU6050.h"
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

Simple_MPU6050 mpu;
ENABLE_MPU_OVERFLOW_PROTECTION();
/*             _________________________________________________________*/
//               X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
//#define OFFSETS  -5260,    6596,    7866,     -45,       5,      -9  // My Last offsets. 
//       You will want to use your own as these are only for my specific MPU6050.
/*             _________________________________________________________*/

//***************************************************************************************
//******************                Print Funcitons                **********************
//***************************************************************************************

#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis()) // (BLACK BOX) Ya, don't complain that I used "for(;;){}" instead of "if(){}" for my Blink Without Delay Timer macro. It works nicely!!!

/* printfloatx() is a helper Macro used with the Serial class to simplify my code and provide enhanced viewing of Float and interger values:
   usage: printfloatx(Name,Variable,Spaces,Precision,EndTxt);
   Name and EndTxt are just char arrays
   Variable is any numerical value byte, int, long and float
   Spaces is the number of spaces the floating point number could possibly take up including +- and decimal point.
   Percision is the number of digits after the decimal point set to zero for interger 
*/
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);//Name,Variable,Spaces,Precision,EndTxt

/* Control Define */

#include "BTS7960.h"
/* Define Mode
  0: Locked Mode
  1: Auto Handling Mode
  2: Remode Handling Mode
*/
/*============ Pins Table============ */

/*  RF Signal                       Mega2560
      CH1                               40
      CH2                               41
      CH3                               42
      CH4                               43
*/

/* Water Pump                       Mega2560
      waterPump                         39
*/

/* BTS7960 DC Motor Controller      Mega2560
    EN                                  8
    L_PWM                               9
    R_PWM                               10

*/

/* BTS7960 DC Servo Controller       Mega2560
    PWA                                 5
    AIN1                                6
    AIN2                                7
*/

/* Encoder Feedback Input             Mega2560
    ENCA (ChannelA) - Interrupt         2
    ENCB (ChannelB) - Normal            4
*/

/* ================================== */

/* Define Signal Channel RF Pins*/
#define channel2  45        // DC Channel
#define channel3  46        // DC Pump Channel
#define channel4  47        // DC Servo Channel

unsigned long duration2, duration3, duration4;
int32_t   pulse1, // RUN_FW
          pulse2, // BACK_RV
          pulse4; // DC_SERVO
/* Define Water Pump Pins*/
#define waterPump 39

/* Define DIR DC Motor Pins*/
/* DIR: RUN_Forward (RUN_FW): R_PWM - Run Reverse (RUN_RV): L_PWM */
#define EN       8
#define L_PWM    9
#define R_PWM   10
BTS7960 motorController(EN, L_PWM, R_PWM);

/* Define DC Servo Motor Pins */
#define EN0         5
#define L_PWM0      6
#define R_PWM0      7
BTS7960 motorController0(EN0, L_PWM0, R_PWM0);

/* Define Encoders Pins */
#define ECD_A        18
#define ECD_B        4

/* DC Servo Motor Timing */


/* PID Parameter Configuration */
int32_t curPos = 0, desPos = 0, err = 0;


int PrintValues(int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);
    Serial.printfloatx(F("Yaw")  , ypr[0], 9, 4, F(", ")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    Serial.printfloatx(F("Pitch"), ypr[1], 9, 4, F(", "));
    Serial.printfloatx(F("Roll") , ypr[2], 9, 4, F("\n"));
  }
}

int ChartValues(int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);
    Serial.printfloatx("", ypr[0], 9, 4, F(",")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    Serial.printfloatx("", ypr[1], 9, 4, F(","));
    Serial.printfloatx("", ypr[2], 9, 4, F("\n"));
  }
}

//Gyro, Accel and Quaternion
int PrintAllValues(int16_t *gyro, int16_t *accel, int32_t *quat, uint16_t SpamDelay = 200) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);
    Serial.printfloatx(F("Yaw")  , xyz[0], 9, 4, F(",   ")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    //int32_t setpoint = int32_t(xyz[0]);
    //Serial.print("Goc: ");
    //Serial.print(setpoint);
    read_channel4_auto(xyz[0]);

    //Serial.printfloatx(F("Pitch"), xyz[1], 9, 4, F(",   "));
    //Serial.printfloatx(F("Roll") , xyz[2], 9, 4, F(",   "));
    /*
      Serial.printfloatx(F("ax")   , accel[0], 5, 0, F(",   "));
      Serial.printfloatx(F("ay")   , accel[1], 5, 0, F(",   "));
      Serial.printfloatx(F("az")   , accel[2], 5, 0, F(",   "));
      Serial.printfloatx(F("gx")   , gyro[0],  5, 0, F(",   "));
      Serial.printfloatx(F("gy")   , gyro[1],  5, 0, F(",   "));
      Serial.printfloatx(F("gz")   , gyro[2],  5, 0, F("\n"));
    */
    Serial.println();
  }
}

int ChartAllValues(int16_t *gyro, int16_t *accel, int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);
    Serial.printfloatx("", ypr[0], 9, 4, F(",")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    Serial.printfloatx("", ypr[1], 9, 4, F(","));
    Serial.printfloatx("", ypr[2], 9, 4, F(", "));
    Serial.printfloatx("", accel[0], 5, 0, F(","));
    Serial.printfloatx("", accel[1], 5, 0, F(","));
    Serial.printfloatx("", accel[2], 5, 0, F(","));
    Serial.printfloatx("", gyro[0],  5, 0, F(","));
    Serial.printfloatx("", gyro[1],  5, 0, F(","));
    Serial.printfloatx("", gyro[2],  5, 0, F("\n"));
  }
}

int PrintQuaternion(int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    Serial.printfloatx(F("quat w")  , q.w, 9, 4, F(",   ")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    Serial.printfloatx(F("x")       , q.x, 9, 4, F(",   "));
    Serial.printfloatx(F("y")       , q.y, 9, 4, F(",   "));
    Serial.printfloatx(F("z")       , q.z, 9, 4, F("\n"));
  }
}

int PrintEuler(int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  float euler[3];         // [psi, theta, phi]    Euler angle container
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetEuler(euler, &q);
    Serial.printfloatx(F("euler  ")  , euler[0] * 180 / M_PI, 9, 4, F(",   ")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    Serial.printfloatx(F("")       , euler[1] * 180 / M_PI, 9, 4, F(",   "));
    Serial.printfloatx(F("")       , euler[2] * 180 / M_PI, 9, 4, F("\n"));
  }
}

int PrintRealAccel(int16_t *accel, int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  VectorFloat gravity;
  VectorInt16 aa, aaReal;
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.SetAccel(&aa, accel);
    mpu.GetLinearAccel(&aaReal, &aa, &gravity);
    Serial.printfloatx(F("aReal x")  , aaReal.x , 9, 4, F(",   ")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    Serial.printfloatx(F("y")        , aaReal.y , 9, 4, F(",   "));
    Serial.printfloatx(F("z")        , aaReal.z, 9, 4, F("\n"));
  }
}


int PrintWorldAccel(int16_t *accel, int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  VectorFloat gravity;
  VectorInt16 aa, aaReal, aaWorld;
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.SetAccel(&aa, accel);
    mpu.GetLinearAccel(&aaReal, &aa, &gravity);
    mpu.GetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.printfloatx(F("aWorld x")  , aaWorld.x , 9, 4, F(",   ")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    Serial.printfloatx(F("y")        , aaWorld.y, 9, 4, F(",   "));
    Serial.printfloatx(F("z")        , aaWorld.z, 9, 4, F("\n"));
  }
}
//***************************************************************************************
//******************              Callback Funciton                **********************
//***************************************************************************************


void print_Values (int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {
  uint8_t Spam_Delay = 100; // Built in Blink without delay timer preventing Serial.print SPAM

  // PrintValues(quat, Spam_Delay);
  // ChartValues(quat, Spam_Delay);
  PrintAllValues(gyro, accel, quat, Spam_Delay);
  // ChartAllValues(gyro, accel, quat, Spam_Delay);
  // PrintQuaternion(quat, Spam_Delay);
  // PrintEuler(quat, Spam_Delay);
  // PrintRealAccel(accel, quat,  Spam_Delay);
  // PrintWorldAccel(accel, quat, Spam_Delay);
}

//***************************************************************************************
//******************                Setup and Loop                 **********************
//***************************************************************************************
void setup() {
  init_RF();
  //init_WATER_pump();
  uint8_t val;
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(100000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  // initialize serial communication
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println(F("Start:"));
#ifdef OFFSETS
  Serial.println(F("Using Offsets"));
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).load_DMP_Image(OFFSETS); // Does it all for you
  init_Encoders();

#else
  /*//Serial.println(F(" Since no offsets are defined we aregoing to calibrate this specific MPU6050,\n"
                   " Start by having the MPU6050 placed stationary on a flat surface to get a proper accellerometer calibration\n"
                   " Place the new offsets on the #define OFFSETS... line at top of program for super quick startup\n\n"
                   " \t\t\t[Press Any Key]"));*/
  //while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again
  delay(500);
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).CalibrateMPU().load_DMP_Image();// Does it all for you with Calibration
#endif
  mpu.on_FIFO(print_Values);
}

void loop() {
  mpu.dmp_read_fifo();
}

void init_RF() {
  pinMode(channel2, INPUT);
  pinMode(channel3, INPUT);
  pinMode(channel4, INPUT);
}

void init_WATER_pump() {
  pinMode(waterPump, OUTPUT);
  digitalWrite(waterPump, LOW);
}

void init_Encoders() {
  digitalWrite(ECD_A, HIGH);
  digitalWrite(ECD_B, HIGH);
  attachInterrupt(digitalPinToInterrupt(ECD_A), [] {
    if (digitalRead(ECD_B)) {
      curPos--;
    }
    else{
      curPos++;
    }
  }, RISING);
  Serial.setTimeout(150);
}

void WATER_pump_run() {
  digitalWrite(waterPump, HIGH);
}

void WATER_pump_brake() {
  digitalWrite(waterPump, LOW);
}

void DC_motor_run( uint32_t pulse, uint32_t dir ) {
  motorController.Enable();
  if (dir == 1) {
    motorController.TurnLeft(pulse);
  }
  else if (dir == 0) {
    motorController.TurnRight(pulse);
  }
}

void DC_motor_brake() {
  motorController.Disable();
}

void DC_SERVO_run(float value)
{
  value = constrain(value, -255.00, 255.00);
  motorController0.Enable();
  if (value > 0)
  {
    motorController0.TurnRight(value);
    //Serial.println(value);
  }
  else
  {
    motorController0.TurnLeft(-value);

    //Serial.println(-value);
  }
}

void DC_SERVO_brake() {
  motorController0.Disable();
}

float partP(float err, float p)
{
  return err * p;
}

float partD(float err, float d)
{
  static float preErr;
  uint32_t preTime = 0;
  float dErr, ret;

  dErr = err - preErr;
  preErr = err;
  ret = dErr * d / float(millis() - preTime);
  preTime = millis();
  return ret;
}

float partI(float err, float i)
{
  static float sum;
  float ret;
  static uint32_t preTime = 0;

  if (abs(err) < 500)
    sum += err;
  ret = sum * i * float(millis() - preTime);
  preTime = millis();

  return ret;
}

void read_channel2() {
  duration2 = pulseIn(channel2, HIGH);
  if (duration2 >= 1473 && duration2 <= 1910) {
    duration2 = constrain(duration2, 1473, 1910);
    /* Min: 1485 Max: 1910*/
    if ( duration2 >= 1473 && duration2 <= 1473 ) {
      pulse1 = map(duration2, 1473, 1050, 0, 255);
      //Serial.println(pulse1);
      DC_motor_run(pulse1, 1); // RUN_FW
    }

    else if ( duration2 >= 1485 && duration2 <= 1910 ) {
      pulse1 = map(duration2, 1485, 1910, 0, 255);
      //Serial.println(pulse2);
      DC_motor_run(pulse1, 0); // RUN_RV
    }
  }
}

void read_channel3() {
  duration3 = pulseIn(channel3, HIGH);
  duration3 = constrain(duration3, 1050, 1890);
  if ( duration3 >= 1050 && duration3 <= 1473  ) {
    WATER_pump_run();
  }
  else if ( duration3 >= 1485 && duration3 <= 1910 ) {
    WATER_pump_brake();
  }
}

void read_channel4() {
  duration4 = pulseIn(channel4, HIGH); // DC Servo Motor
  //Serial.println(duration4);
  duration4 = constrain(duration4, 1050, 1890);
  //Serial.println(duration4);
  //Serial.println(duration4);
  duration4 = map(duration4, 1050, 1890, -5, 5);
  desPos = duration4 * 6553;
  
  if (desPos <= -32000) {
    desPos = -32000;
  }
  else if (desPos >= 32000) {
    desPos = 32000;
  }

  err = desPos - curPos;
  //DC_SERVO_run(partP(err, 0.099) +  + partI(err, 0.0001) + partD(err,0));
  DC_SERVO_run(partP(err, 0.049) +  + partI(err, 0.0001) + partD(err, 0));
  //Serial.println(desPos);
  Serial.println(curPos);
}

void read_channel4_auto(float setpoint){
  //Serial.println();
  desPos = (int32_t)setpoint * 277;
  
  if (desPos <= -16620) {
    desPos = -16620;
  }
  else if (desPos >= 16620) {
    desPos = 16620;
  }
  else
   desPos = desPos;
  Serial.println(desPos);
  err = desPos - curPos;
  DC_SERVO_run(partP(err, 0.099) +  + partI(err, 0.0001) + partD(err,0));
  //DC_SERVO_run(partP(err, 0.049) +  + partI(err, 0.0001) + partD(err, 0));
  //Serial.println(desPos);
  Serial.println(curPos); 
}
