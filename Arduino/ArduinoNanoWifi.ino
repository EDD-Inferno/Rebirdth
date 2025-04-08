#include <SPI.h>
#include <WiFiNINA.h>

const char* ssid = "Kaden is Awesome";   // Your WiFi SSID
const char* password = "kadeniscool";    // Your WiFi password
const char* serverIP = "172.20.10.2";    // Replace with your computer's local IP
const int serverPort = 1234;             // Same port as the Python server

const int MOSFET_PIN = 5;  // Pin controlling the MOSFET

WiFiClient client;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    pinMode(MOSFET_PIN, OUTPUT);
    digitalWrite(MOSFET_PIN, LOW);  // Default: MOSFET OFF

    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("\nConnected!");

    Serial.println("Waiting for server commands...");
}

void loop() {
    if (client.connect(serverIP, serverPort)) {
        Serial.println("Connected to server, waiting for command...");

        // Allow time for data to be sent
        delay(500);  // Allow time for Python server to send the response

        // Wait for response
        String response = "";
        while (client.available()) {
            char c = client.read();
            response += c;
        }
        response.trim(); // Remove extra spaces/newlines

        // Debugging: Print the response the Arduino receives
        Serial.print("Received from server: '");
        Serial.print(response);
        Serial.println("'");

        // Control the MOSFET based on response
        if (response.equals("TURN ON")) {
            digitalWrite(MOSFET_PIN, HIGH);
            Serial.println("MOSFET TURNED ON");
        } else if (response.equals("TURN OFF")) {
            digitalWrite(MOSFET_PIN, LOW);
            Serial.println("MOSFET TURNED OFF");
        } else {
            Serial.println("Unknown response.");
        }

        client.stop();
    } else {
        Serial.println("Connection failed.");
    }

    delay(500);  // Small delay before trying again
}
