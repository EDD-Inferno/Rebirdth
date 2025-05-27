/*
  Pin assignments:
   - Laser Diode (980MD-30-0813) on DIO3
   - IR Receiver (TSOP13238) on DIO2
   - LED on DIO11
*/
 
#include <WiFi.h>
#include <WiFiUdp.h>

// -------------------------
// 1) Pin Definitions
// -------------------------
const int LED_PIN    = 9;
const int IR_PIN     = 4;


const char* ssid = "Benjamin's iPhone";
const char* password = "Infinitesimals";

WiFiUDP udp;
const char* raspberryPiIP = "192.168.1.100";  // TODO: replace with your Pi's IP
const unsigned int port = 4210;
 
void setup() {
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  // Configure pins
  pinMode(IR_PIN, INPUT);    // TSOP13238 output 
  pinMode(LED_PIN, OUTPUT);

  // (Optional) Begin serial for debug
  Serial.begin(115200);
  Serial.println("\nSTARTING\n---------------------------------------");
}

bool lastState = false;
float onRatio = 0;

uint64_t hit_history;

uint64_t last_time_chunk = 0;
#define chunk_duration 20

void loop() {
  if (millis() - last_time_chunk > chunk_duration) {
    hit_history <<= 1;
  }

  bool state = digitalRead(IR_PIN) == LOW;
  hit_history |= (uint64_t)state;

  if (__builtin_popcountll(hit_history) > 16) {
    analogWrite(LED_PIN, 255);
    while (true) {
      udp.beginPacket(raspberryPiIP, port);
      udp.write("The first tower has been hit.");  // your flag
      udp.endPacket();
      delay(250);
    }
  } else if (state) {
    analogWrite(LED_PIN, 85);
  } else {
    analogWrite(LED_PIN, 0);
  }
  Serial.print(millis());
  Serial.print(" -> ");
  Serial.println(state);
}

