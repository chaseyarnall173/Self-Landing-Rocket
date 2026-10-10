// Host-side STUB of the Arduino/Teensy API so `make compile-check` can
// type-check the flight firmware without the Teensy toolchain. It does NOT
// emulate hardware; it only declares what the firmware uses.
#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define DMAMEM
inline uint32_t millis() { return 0; }
inline uint32_t micros() { return 0; }
inline void delay(uint32_t) {}
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return HIGH; }
inline int analogRead(uint8_t) { return 0; }
inline void analogWrite(uint8_t, uint32_t) {}
inline void analogWriteFrequency(uint8_t, float) {}
inline void analogWriteResolution(int) {}
inline void analogReadResolution(int) {}
inline void tone(uint8_t, unsigned, unsigned long) {}
class HardwareSerial {
public:
    void begin(uint32_t) {}
    int available() { return 0; }
    int read() { return -1; }
    template <typename... A> int printf(const char* f, A... a) { return ::printf(f, a...); }
};
extern HardwareSerial Serial, Serial1;
