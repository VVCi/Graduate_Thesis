#include "BTS7960.h"

unsigned long duration1, duration2, duration3, duration4;
unsigned long   pulse1, // RUN_FW
          pulse2, // BACK_RV
          pulse3, // STEPPER 
          pulse4; // DC_SERVO

#define EN       8
#define L_PWM    9
#define R_PWM   10
BTS7960 motorController(EN, L_PWM, R_PWM);

#define channel1  44
#define channel2  7
#define channel3  46
#define channel4  47

void setup() {
  Serial.begin(9600);
  init_RF();
}

void init_RF() {
  pinMode(channel1, INPUT);
  pinMode(channel2, INPUT);
  pinMode(channel3, INPUT);
  pinMode(channel4, INPUT);
}

void DC_motor_run( uint32_t pulse, uint32_t dir ) {
  motorController.Enable();
  if (dir == 1) {
    motorController.TurnLeft(pulse);
    Serial.println(pulse);
  }
  else if (dir == 0) {
    motorController.TurnRight(pulse);
    Serial.println(pulse);
  }
}

void read_channel2() {
  duration2 = pulseIn(channel2, HIGH); // DC Motor
  duration2 = constrain(duration2, 1040, 1910);
  /* Min: 1485 Max: 1910*/
  if ( duration2 >= 1040 && duration2 <= 1473 ) {
    pulse1 = map(duration2, 1473, 1050, 0, 255);
    DC_motor_run(pulse1, 1); // RUN_FW
  }
  else if ( duration2 >= 1485 && duration2 <= 1910 ) {
    pulse2 = map(duration2, 1485, 1910, 0, 255);
    DC_motor_run(pulse2, 0); // RUN_RV
  }
}

void loop() {
  /* DC Motor Handle */
  read_channel2();

}
