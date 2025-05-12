#include "Arduino.h"

// Mock pin states
static uint8_t pin_modes[100] = {0};
static uint8_t digital_pins[100] = {0};
static int analog_pins[100] = {0};
unsigned long current_millis = 0;

HardwareSerial Serial;

void pinMode(uint8_t pin, uint8_t mode) {
    if (pin < 100) {
        pin_modes[pin] = mode;
    }
}

void digitalWrite(uint8_t pin, uint8_t val) {
    if (pin < 100) {
        digital_pins[pin] = val;
    }
}

int digitalRead(uint8_t pin) {
    if (pin < 100) {
        return digital_pins[pin];
    }
    return LOW;
}

int analogRead(uint8_t pin) {
    if (pin < 100) {
        return analog_pins[pin];
    }
    return 0;
}

void analogWrite(uint8_t pin, int val) {
    if (pin < 100) {
        analog_pins[pin] = val;
    }
}

void delay(unsigned long ms) {
    current_millis += ms;
}

unsigned long millis(void) {
    return current_millis;
}
