#include <Wire.h>
#include <MPU6050_tockn.h>
#include "PID.h"

// Actuator Control Pins
#define ACT1_EXTEND 30  // Actuator 1 Extend (PWM)
#define ACT1_RETRACT 28 // Actuator 1 Retract (PWM)
#define ACT1_CON 3 // Actuator 1 Retract (PWM)

#define ACT2_EXTEND 32  // Actuator 2 Extend (PWM)
#define ACT2_RETRACT 34 // Actuator 2 Retract (PWM)
#define ACT2_CON 2 // Actuator 1 Retract (PWM)

#define MIN_ACTUATOR_OUTPUT 48
#define TOLERANCE 0.5


// IMU Setup
MPU6050 mpu(Wire);
unsigned long timer = 0;

// PID Controllers (Pitch & Roll)
// If Too Sluggish - Increase Kp by 0.2 increments
// If Overshoots - Increase Kd by 0.1 increments
// If Steady Error - Increase Ki by 0.01 increments

PIDConfig pitchConfig(128.0f, 2.f, 0.2f, 255.0f, 2, 128);  // kp, ki, kd
PID pitchPID(pitchConfig);

PIDConfig rollConfig(64.f, 2.f, 0.2f, 255, 2, 128);  // kp, ki, kd
PID rollPID(rollConfig);


void setup() {
    Serial.begin(9600);
    Wire.begin();

    // Initialize MPU6050
    mpu.begin();
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    mpu.calcGyroOffsets(true);
    Serial.println("Done!n");

    // Set actuator control pins
    pinMode(ACT1_EXTEND, OUTPUT);
    pinMode(ACT1_RETRACT, OUTPUT);
    pinMode(ACT2_EXTEND, OUTPUT);
    pinMode(ACT2_RETRACT, OUTPUT);

    Serial.println("System Ready");
}

void loop() {
    mpu.update();
    float pitch = -mpu.getAngleX();
    // Serial.print("X : ");
    // Serial.print(mpu.getAngleX());
    float roll = mpu.getAngleY();
    // Serial.print(" Y : ");
    // Serial.println(mpu.getAngleY());
    timer = millis();

    // Compute PID adjustments
    // pitch changes SAME as error
    // roll changes OPPOSITE of error
    
    float pitchAdjustment = pitchPID.compute(pitch); // Invert if needed
    float rollAdjustment = rollPID.compute(roll);

    // conserve output ratio
    auto total_error = abs(pitch) + abs(roll);

    auto output1 = pitchAdjustment - rollAdjustment;
    auto output2 = pitchAdjustment + rollAdjustment;
    auto max_output = max(abs(output1), abs(output2));

    if (max_output > 255) {
        auto ratio = 255.f / max_output;
        output1 *= ratio;
        output2 *= ratio;
    }

    // control actuators
    controlActuator(ACT1_EXTEND, ACT1_RETRACT, ACT1_CON, output1, total_error);  // Actuator 1 controls roll
    controlActuator(ACT2_EXTEND, ACT2_RETRACT, ACT2_CON, output2, total_error); // Actuator 2 controls pitch

    // Print output
    Serial.print("Pitch: "); Serial.print(pitch);
    Serial.print(" | Roll: "); Serial.print(roll);
    Serial.print(" | PID Out: "); Serial.print(pitchAdjustment);
    Serial.print(", "); Serial.println(rollAdjustment);

    delay(20); // ~50Hz control loop
}

// Controls a single actuator based on PID output
void controlActuator(int extendPin, int retractPin, int conPin, float pidOutput, float error) {
    float output = max(MIN_ACTUATOR_OUTPUT, min(fabsf(pidOutput), 255.f));
    if (error > TOLERANCE) {
        analogWrite(conPin, output); // Extend (PWM)
        if (pidOutput > 0) {
            digitalWrite(extendPin, HIGH);
            digitalWrite(retractPin, LOW);
        } else {
            digitalWrite(extendPin, LOW);
            digitalWrite(retractPin, HIGH);
        }
    } else {
        digitalWrite(extendPin, LOW); // Stop
        digitalWrite(retractPin, LOW);
    }
}