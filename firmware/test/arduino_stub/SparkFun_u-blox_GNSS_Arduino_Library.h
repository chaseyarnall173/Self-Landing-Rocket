#pragma once
#include "Wire.h"
#define COM_TYPE_UBX 1
class SFE_UBLOX_GNSS {
public:
    bool begin(TwoWire&) { return true; }
    bool setI2COutput(uint8_t) { return true; }
    bool setNavigationFrequency(uint8_t) { return true; }
    bool setAutoPVT(bool) { return true; }
    bool getPVT() { return false; }
    int32_t getNedEastVel() { return 0; }
    int32_t getNedNorthVel() { return 0; }
};
