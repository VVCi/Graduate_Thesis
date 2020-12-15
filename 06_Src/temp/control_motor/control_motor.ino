#define CW_PIN 5
#define CCW_PIN 6

#define ECD_A 2
#define ECD_B 4
int32_t curPos = 0, desPos = 200, err = 0;
uint32_t preTime = 0;
void pwmMotor(float value)
{
  value = constrain(value, -255.0, 255.0);
  if (value > 0)
  {
    analogWrite(CW_PIN, 0);
    analogWrite(CCW_PIN, value);
  }
  else
  {
    analogWrite(CCW_PIN, 0);
    analogWrite(CW_PIN, - value);
  }
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

  if (abs(err) < 20) // các bạn có thể giới hạn khâu I bằng cách này.
    sum += err;
  ret = sum * i * float(millis() - preTime);
  preTime = millis();
//  ret = constrain(ret, -150.0, 150.0); các bạn có thể giới hạn khâu I bằng cách này cũng được.
  
  return ret;
}
void setup() {
  // put your setup code here, to run once:
  digitalWrite(ECD_A, HIGH);
  digitalWrite(ECD_B, HIGH);
  attachInterrupt(digitalPinToInterrupt(ECD_A), [] {
    if (digitalRead(ECD_B))
      curPos--;
    else
      curPos++;
  }, RISING);
  Serial.begin(9600);
  Serial.setTimeout(111);
//  preTime = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0)
  {
    desPos = Serial.parseInt();
    Serial.readString();
  }
  err = desPos - curPos;
  pwmMotor(partP(err, 3) + partD(err, 400) + partI(err, .1));
  Serial.println(curPos);
//if(millis() - preTime > 5000)
//{
//  desPos+=500;
//  preTime = millis();
//}


}
