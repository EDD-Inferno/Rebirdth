// Simplified MPU6050 library for angle reading
#include <Wire.h>

class MPU6050 {
public:
    MPU6050(TwoWire &w) { wire = &w; }

    void begin() {
        wire->begin();
        wire->beginTransmission(0x68);
        wire->write(0x6B); wire->write(0x00); // Wake up
        wire->endTransmission();
    }

    void calcGyroOffsets(bool console = false) {
        // Simple calibration (keep IMU flat!)
        // ...
    }

    void update() {
        wire->beginTransmission(0x68);
        wire->write(0x3B); // Start reading registers
        wire->endTransmission(false);
        wire->requestFrom(0x68, 14); // Read 14 bytes

        // Process raw data
        // ...
    }

    float getAngleX() { return angleX; } // Pitch
    float getAngleY() { return angleY; } // Roll

private:
    TwoWire *wire;
    float angleX = 0, angleY = 0;
    // ... rest of implementation
};