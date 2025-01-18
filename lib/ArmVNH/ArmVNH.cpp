#include "ArmVNH.h"

ArmVNH::ArmVNH(const uint8_t pwmPin, const uint8_t forwardPin, const uint8_t reversePin, PCF8574 *iox) {
    m_pwmPin = pwmPin;
    m_forwardPin = forwardPin;
    m_reversePin = reversePin;
    m_iox = iox;
    m_hasCS = false;
}

ArmVNH::ArmVNH(const uint8_t pwmPin, const uint8_t forwardPin, const uint8_t reversePin, const uint8_t csPin, PCF8574 *iox) {
    m_pwmPin = pwmPin;
    m_forwardPin = forwardPin;
    m_reversePin = reversePin;
    m_iox = iox;
    m_csPin = csPin;
    m_hasCS = true;
}


#if defined(ARDUINO)
#include <Arduino.h>

void ArmVNH::init() {
    pinMode(m_pwmPin, OUTPUT);

    // using iox.read8() elsewhere might affect the state of the io expander
    // TODO: TEST ON HARDWARE
    // m_iox->setButtonMask(m_iox->getButtonMask() | (1 << m_forwardPin) | (1 << m_reversePin));

    m_iox->write(m_forwardPin, LOW);
    m_iox->write(m_reversePin, LOW);
    
    analogWrite(m_pwmPin, 0);

    if (m_hasCS) {
        pinMode(m_csPin, INPUT);
    }
}

void ArmVNH::configFrequency(const float frequency) {
    analogWriteFrequency(m_pwmPin, frequency);
}

void ArmVNH::configCSScale(float gain) {
    m_csGain = gain * 3.3 / 1023.0; // convert to ADC scale instead of volts
}

float ArmVNH::readCurrent() {
    delayNanoseconds(50);
    float current = analogRead(m_csPin) * m_csGain;

    return current;
}

void ArmVNH::drive(int16_t decipercent) const {
    int32_t rampedDecipercent = applyConfigs(decipercent);
    uint8_t pwm = abs(rampedDecipercent) * 255 / 1000;

    if (rampedDecipercent > 0) {
        m_iox->write(m_forwardPin, HIGH);
        m_iox->write(m_reversePin, LOW);
    } else if (rampedDecipercent < 0) {
        m_iox->write(m_forwardPin, LOW);
        m_iox->write(m_reversePin, HIGH);
    } else {
        m_iox->write(m_forwardPin, LOW);
        m_iox->write(m_reversePin, LOW);
    }

    analogWrite(m_pwmPin, pwm);
}


#endif