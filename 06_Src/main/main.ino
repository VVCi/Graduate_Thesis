#include "BTS7960.h"
/* Define Mode
  0: Locked Mode
  1: Auto Handling Mode
  2: Remode Handling Mode
*/
/*============ Pins Table============ */

/*  RF Signal                       Mega2560
      CH2                               45
      CH4                               47
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
    ENCA (ChannelA) - Interrupt         18
    ENCB (ChannelB) - Normal            4
*/

/* ================================== */

/* Define Signal Channel RF Pins*/
#define channel2  45        // DC Channel
#define channel4  47        // DC Servo Channel

unsigned long duration2, duration3, duration4;
int32_t   pulse1, // RUN_FW
          pulse2, // BACK_RV
          pulse4, // DC_SERVO
          pulse5,
          pulse6;
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
const float degree = 100000 / 360;


/* DC Servo Motor Timing */

/* PID Parameter Configuration */
int32_t curPos = 0, desPos = 0, err = 0;

void setup() {
  Serial.begin(4800);
  init_RF();
  delay(200);
  //init_WATER_pump();
  init_Encoders();
}

void loop() {
  /*Mode Camera Handlde*/
  /*if (Serial.available() > 0)
    {
    //Testing Serial
    desPos = Serial.parseInt();
      Serial.readString();
      if (desPos >= 40000)
      desPos = 40000;
      if (desPos <= -40000)
      desPos = -40000;
    }*/

  /* DC_Handle */
  read_channel2();
  //duration2 = pulseIn(channel2, HIGH);
  /* Servo_Handle */
  read_channel4();

  //err = desPos - curPos;
  //DC_SERVO_run(partP(err, 0.198) +  + partI(err, 0) + partD(err, 0)); //1
  //DC_SERVO_run(partP(err, 0.099) +  + partI(err, 0.0001) + partD(err, 0)); //2
  //DC_SERVO_run(partP(err, 0.099) +  + partI(err, 0.001) + partD(err, 0)); //3
  //DC_SERVO_run(partP(err, 0.099) +  + partI(err, 0.00011) + partD(err, 0)); //4
  //DC_SERVO_run(partP(err, 0.099) +  + partI(err, 0.0001) + partD(err,0));
  //DC_SERVO_run(partP(err, 0.049) +  + partI(err, 0.0001) + partD(err, 0));
  //Serial.println(desPos);
  //Serial.println(curPos);
}

void init_RF() {
  pinMode(channel2, INPUT);
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
  Serial.setTimeout(111);
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
  if (duration2 >= 1050 && duration2 <= 1910) {
    duration2 = constrain(duration2, 1050, 1910);
    /* Min: 1485 Max: 1910*/
    if ( duration2 >= 1050 && duration2 <= 1473 ) {
      pulse1 = map(duration2, 1473, 1050, 0, 255);
      //Serial.println(pulse1);
      DC_motor_run(pulse1, 1); // RUN_FW
    }
    else if ( duration2 >= 1485 && duration2 <= 1910 ) {
      pulse2 = map(duration2, 1485, 1910, 0, 255);
      //Serial.println(pulse2);
      DC_motor_run(pulse2, 0); // RUN_RV
    }
  }
}

void read_channel3() {
  duration4 = pulseIn(channel4, HIGH);
  if (duration4 >= 1050 && duration4 <= 1910) {
    duration4 = constrain(duration4, 1050, 1910);
    /* Min: 1485 Max: 1910*/
    if ( duration4 >= 1050 && duration4 <= 1473 ) {
      pulse4 = map(duration4, 1473, 1050, 0, 255);
      Serial.println(pulse1);
      DC_SERVO_run(pulse4); // RUN_FW
    }
    else if ( duration4 >= 1485 && duration4 <= 1910 ) {
      pulse5 = map(duration4, 1485, 1910, 0, 255);
      Serial.println(pulse2);
      DC_SERVO_run(-pulse1); // RUN_RV
    }
  }
}

void read_channel4() {
  duration4 = pulseIn(channel4, HIGH); // DC Servo Motor
  //Serial.println(duration4);
  duration4 = constrain(duration4, 1030, 1890);
  //Serial.println(duration4);
  //Serial.println(duration4);
  duration4 = map(duration4, 1030, 1890, -5, 5);
  desPos = duration4 * 1939;

  if (curPos <= -9695) {
    //desPos = -9695;
    //desPos = 0;
    DC_SERVO_brake();
  }
  else if (curPos >= 9695) {
    //desPos = 0;
    DC_SERVO_brake();
  }
  else {
    err = desPos - curPos;
    DC_SERVO_run(partP(err, 0.01) +  + partI(err, 0) + partD(err, 0));
    Serial.println(desPos);
    Serial.println(curPos);
  }


  //if ((desPos >= -9695) && (desPos <= 9695)){

  //DC_SERVO_run(partP(err, 0.099) +  + partI(err, 0.0001) + partD(err,0));

  //}

}
