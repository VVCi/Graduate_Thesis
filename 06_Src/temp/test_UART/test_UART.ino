String incomming_UART;
char char_range[10];
int int_range;
byte blank;

void setup() {
  Serial.begin(115200);

}

void UART_handle() {
  if (Serial.available() > 0) {
    incomming_UART = Serial.read();
  }
  for (int i = 0; i < incomming_UART.length(); i++) {
    if (incomming_UART.charAt(i) == '1'){
      char_range = incomming_UART.charAt(i);
      break;
    } 
  }
  

  Serial.println(char_range);

}

void loop() {
  UART_handle();

}
