#include "MPU9250.h"

MPU9250 mpu, mpu2;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    pinMode(6, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(2, OUTPUT);
    digitalWrite(6, LOW);
    digitalWrite(3, HIGH);
    digitalWrite(2, HIGH);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    digitalWrite(6, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(2, LOW);
    if (!mpu2.setup(0x69)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
}

void loop() {
    digitalWrite(6, LOW);
    digitalWrite(3, HIGH);
    digitalWrite(2, HIGH);
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 100) {
            print_roll_pitch_yaw();
            prev_ms = millis();
        }
    }

    digitalWrite(6, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(2, LOW);

    if (mpu2.update()) {
        static uint32_t prev_ms2 = millis();
        if (millis() > prev_ms2 + 100) {
            print2_roll_pitch_yaw();
            prev_ms2 = millis();
        }
    }
}

void print_roll_pitch_yaw() {
    Serial.print("Yaw , Pitch , Roll : ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}

void print2_roll_pitch_yaw() {
    Serial.print("Yaw2, Pitch2, Roll2: ");
    Serial.print(mpu2.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu2.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu2.getRoll(), 2);
}
