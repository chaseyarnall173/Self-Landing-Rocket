#pragma once
#include "Arduino.h"
#define BUILTIN_SDCARD 254
#define FILE_WRITE 1
class File {
public:
    explicit operator bool() const { return true; }
    void println(const char*) {}
    template <typename... A> int printf(const char*, A...) { return 0; }
    size_t write(const uint8_t*, size_t n) { return n; }
    void close() {}
};
class SDClass {
public:
    bool begin(uint8_t) { return true; }
    bool exists(const char*) { return false; }
    File open(const char*, int) { return File(); }
};
extern SDClass SD;
