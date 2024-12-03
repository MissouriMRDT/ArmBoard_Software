#include "Arm_Software2025.h"

#include <cmath>

void setup() {
    Serial.begin(115200);
    Serial.println("Setup");

    // Button pins
    pinMode(B_ENC_0, INPUT);
    pinMode(B_ENC_1, INPUT);
    pinMode(B_ENC_2, INPUT);
    pinMode(B_ENC_3, INPUT);

    // IO expander pins
    IOX_TWI.begin();
    IOX1.begin();
    IOX2.begin();
    IOX3.begin();
}
