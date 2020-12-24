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

/*
    TB6560 Stepper Controller       Mega2560
     CLK+ (stepPin)                     23
     CW+  (dirPin)                      24
     EN+  (enPin)                       25
     CLK-                               GND
     CW-                                GND
     EN-                                GND
     LOW SW Journey                     26
     HIGH SW Journey                    27
*/

/* ================================== */

/* Define Signal Channel RF Pins*/
#define channel1  44        // DC Stepper Channel 
#define channel2  45        // DC Channel
#define channel3  46        // DC Pump Channel
#define channel4  47        // DC Servo Channel

unsigned long duration1, duration2, duration3, duration4;
int32_t   pulse1, // RUN_FW
          pulse2, // BACK_RV
          pulse3, // STEPPER
          pulse4; // DC_SERVO
String incomming_UART; char char_range;
byte blank;
int32_t int_range;
/* Define Water Pump Pins*/
#define waterPump 39

/* Define DIR DC Motor Pins*/
/* DIR: RUN_Forward (RUN_FW): R_PWM - Run Reverse (RUN_RV): L_PWM */
#define EN       8
#define L_PWM    9
#define R_PWM   10
BTS7960 motorController(EN, L_PWM, R_PWM);

/* Define Stepper Motor Pins */
#define stepPin    23
#define dirPin     24
#define enPin      25

/* Define Switch Journey Pins*/
#define swLow      26
#define swHigh     27

/* Define DC Servo Motor Pins */
#define EN0         5
#define L_PWM0      6
#define R_PWM0      7
BTS7960 motorController0(EN0, L_PWM0, R_PWM0);

/* Define Encoders Pins */
#define ECD_A        2
#define ECD_B        4

/* DC Servo Motor Timing */


/* PID Parameter Configuration */
int32_t curPos = 0, desPos = 0, err = 0;

void setup() {
  Serial.begin(115200);
  init_RF();
  //init_WATER_pump();
  //init_SW();
  //init_STEP_motor();
  init_Encoders();
}

void loop() {
  /*Mode Camera Handlde*/
  if (Serial.available() > 0)
  {
    /* Income Signal */
    //String incoming_UART = Serial.read();

    /* Testing Serial */
    /*desPos = Serial.parseInt();
      Serial.readString();
      if (desPos >= 40000)
      desPos = 40000;
      if (desPos <= -40000)
      desPos = -40000;*/
  }

  /* DC_Handle */
  read_channel2();
  /* Servo_Handle */
  //read_channel4();

  /* Pump Handle */
  //read_channel3();
}

float PIDCompute(float Kp, float Ki, float Kd, float err) {
  float a0, a1, a2, out ;
  static float preErr, pre_preErr, preOut;
  static uint32_t preTime = 0;

  a0 = Kp + (0.5 * (Ki * float(millis() - preTime))) + 0.5 * (Kd / float(millis() - preTime));
  a1 = -Kp + (0.5 * (Ki * float(millis() - preTime))) - (Kd / float(millis() - preTime));
  a2 = 0.5 * (Kd / float(millis() - preTime));
  preTime = millis();

  out = preOut + a0 * err + a1 * preErr + a2 * pre_preErr;

  pre_preErr = preErr;
  preErr = err;
  preOut = out;
  if ((curPos > 0 && curPos > desPos) || (curPos < 0 && curPos < desPos)) {
    curPos = desPos;
  }
  return out;
}

void init_RF() {
  pinMode(channel1, INPUT);
  pinMode(channel2, INPUT);
  pinMode(channel3, INPUT);
  pinMode(channel4, INPUT);
}

void init_WATER_pump() {
  pinMode(waterPump, OUTPUT);
  digitalWrite(waterPump, LOW);
}

void init_SW() {
  pinMode(swLow, INPUT_PULLUP);
  pinMode(swHigh, INPUT_PULLUP);
}

void init_STEP_motor() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, HIGH);
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

void STEP_motor_run( uint32_t cycle, bool dir ) {

  /* STEP Motor Secure Mode*/
  bool enable;
  if ( swLow == 0 || swHigh == 0 ) {
    enable = 0;
  }

  if ( swLow != 0 && swHigh != 0 ) {
    enable = 1;
  }

  /* UP */
  if (dir == 1 && enable == 1) {
    digitalWrite(enPin, LOW);
    digitalWrite(dirPin, HIGH);
    for ( int x = 0; x < 400 * cycle; x++ ) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
    }
  }

  /* DOWN */
  if ( dir == 0 && enable == 1 ) {
    digitalWrite(enPin, LOW);
    digitalWrite(dirPin, LOW);
    for ( int x = 0; x < 400 * cycle; x++ ) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
    }
  }
}

void STEP_motor_brake() {
  digitalWrite(enPin, LOW);
}

void read_channel1() {

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

int UART_handle (String incoming_UART) {
  for (int i = 0; i < incoming_UART.length(); i++) {
    if (incomming_UART.charAt(i) == '1' || incomming_UART.charAt(i) == '2' || incomming_UART.charAt(i) == '3' ||
        incomming_UART.charAt(i) == '4' || incomming_UART.charAt(i) == '5' || incomming_UART.charAt(i) == '6' || incomming_UART.charAt(i) == '7') {
      char_range = incomming_UART.charAt(i);
      //int_range = char_range.toInt();
    }
  }
}
