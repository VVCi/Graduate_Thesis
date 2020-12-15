
String ch;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if(Serial.available() > 0){
    ch = Serial.readString();
    printf("%s\n", ch);  
  }

}
