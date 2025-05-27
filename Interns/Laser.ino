const int LASER_PIN  = 4;
const int IR_LED_PIN  = 5;
const int BUTTON_PIN = 8;

bool laserIsOn = false;

void pulse_laser(bool state) {
  if (state) {
    tone(LASER_PIN, 38000);
    noTone(IR_LED_PIN);
  } else {
    tone(IR_LED_PIN, 38000);
    noTone(LASER_PIN);
  }
}
 
void setup() {
  // Configure pins
  pinMode(LASER_PIN, OUTPUT);
  pinMode(IR_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Use internal pull-up resistor for the button
 
  // Initialize them off
  //digitalWrite(IR_LED_PIN, HIGH);
 
  // (Optional) Begin serial for debug
  Serial.begin(115200);
}

void loop() {
  /*bool buttonState = digitalRead(BUTTON_PIN) == LOW;
  if (buttonState != laserIsOn) {
    laserIsOn = buttonState;
    if (buttonState) {
      Serial.println("Laser");
    } else {
      Serial.println("IR LED");
    }
    pulse_laser(buttonState);
  }
  delay(10);*/
  tone(IR_LED_PIN, 38000, 25);
  delay(50);
}