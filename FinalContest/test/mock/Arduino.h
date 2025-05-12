#ifndef ARDUINO_H
#define ARDUINO_H

#include <stdint.h>
#include <cstddef>
#include <cstring>

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int val);
void delay(unsigned long ms);
unsigned long millis(void);

class Stream {
public:
    virtual size_t write(uint8_t) = 0;
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    virtual void flush() = 0;
};

class HardwareSerial : public Stream {
public:
    void begin(unsigned long baud) { }
    size_t write(uint8_t c) override { return 1; }
    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
    void flush() override { }
};

extern HardwareSerial Serial;

#endif
