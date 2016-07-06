//
// Created by Raul on 4/19/2016.
//

#ifndef COOLSATBARO_COOLSATBARO_H
#define COOLSATBARO_COOLSATBARO_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>



class CoolSatBaro{

public:
    CoolSatBaro();
    long getVal(int address, byte code);
    void initial(uint8_t address);
    void readBaro();
    float getTemp();
    float getPressure();
    float getAltitude();
    double getCorrectedPressure();

private:
    uint32_t D1 = 0;
    uint32_t D2 = 0;
    int64_t dT = 0;
    int32_t TEMP = 0;
    int64_t OFF = 0;
    int64_t SENS = 0;
    int32_t P = 0;
    uint16_t C[7];

    float Temperature;
    float Pressure;
    double correctedPressure;
    float altitude;
    float tempKelvin;
};

#endif //COOLSATBARO_COOLSATBARO_H
