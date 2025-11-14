const int LED_PINS[] = {2, 3, 4, 5, 6, 7, 8, 9}; 
const int NUM_LEDS = 8; 
const int MOTOR_PIN = 10;  

String serial_command; 

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  analogWrite(MOTOR_PIN, 0);
  
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], HIGH); 
  }
  
  Serial.begin(115200); 
  serial_command.reserve(10); 
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read(); 
    if (c == '\n') {
      processCommand(serial_command);
      serial_command = "";
    } else {
      serial_command += c;
    }
  }
}

void processCommand(String cmd) {
  cmd.trim(); 

  if (cmd == "M1") {
    analogWrite(MOTOR_PIN, 100);
    Serial.println("Motor ON");
  } 
  else if (cmd == "M0") {
    analogWrite(MOTOR_PIN, 0);
    Serial.println("Motor OFF");
  }
  else if (cmd.startsWith("L=")) {
    String numStr = cmd.substring(2); 
    int ledCount = numStr.toInt(); 
    setLedCount(ledCount);
    Serial.println("LEDs set to " + String(ledCount));
  }
}

void setLedCount(int count) {
  if (count > NUM_LEDS) {
    count = NUM_LEDS;
  }

  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < count) {
      digitalWrite(LED_PINS[i], LOW); 
    } else {
      digitalWrite(LED_PINS[i], HIGH); 
    }
  }
}