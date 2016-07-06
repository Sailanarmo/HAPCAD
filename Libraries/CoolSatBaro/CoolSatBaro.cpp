//
// Created by Raul on 4/19/2016.
//

#include "CoolSatBaro.h"
//#include "Wire.h"
#define ADDRESS 0x76


CoolSatBaro::CoolSatBaro() {
}

long CoolSatBaro::getVal(int address, byte code) {
    unsigned long ret = 0;
    Wire.beginTransmission(address);
    Wire.write(code);
    Wire.endTransmission();
    delay(10);
    // start read sequence
    Wire.beginTransmission(address);
    Wire.write((byte) 0x00);
    Wire.endTransmission();
    Wire.beginTransmission(address);
    Wire.requestFrom(address, (int)3);
    if (Wire.available() >= 3)
    {
        ret = Wire.read() * (unsigned long)65536 + Wire.read() * (unsigned long)256 + Wire.read();
    }
    else {
        ret = -1;
    }
    Wire.endTransmission();
    return ret;
}

void CoolSatBaro::initial(uint8_t address) {
    Serial3.println();
    Serial3.println("PROM COEFFICIENTS ivan");

    Wire.beginTransmission(address);
    Wire.write(0x1E); // reset
    Wire.endTransmission();
    delay(10);


    for (int i=0; i<6  ; i++) {

        Wire.beginTransmission(address);
        Wire.write(0xA2 + (i * 2));
        Wire.endTransmission();

        Wire.beginTransmission(address);
        Wire.requestFrom(address, (uint8_t) 6);
        delay(1);
        if(Wire.available())
        {
            C[i+1] = Wire.read() << 8 | Wire.read();
        }
        else {
            Serial3.println("Error reading PROM 1"); // error reading the PROM or communicating with the device
        }
        Serial3.println(C[i+1]);
    }
    Serial3.println();
}

void CoolSatBaro::readBaro() {
    D1 = getVal(ADDRESS, 0x48); // Pressure raw
    D2 = getVal(ADDRESS, 0x58);// Temperature raw

    dT   = D2 - ((uint32_t)C[5] * pow(2, 8));
    OFF  = ((int64_t)C[2] * pow(2, 17)) + ((dT * C[4]) / pow(2, 6));
    SENS = ((int32_t)C[1] * pow(2, 16)) + ((dT * C[3]) / pow(2, 7));

    TEMP =  2000 + (int64_t)dT * (int64_t)C[6] / pow(2, 23);

    if(TEMP < 2000) // if temperature lower than 20 Celsius
    {
        int32_t T1    = 0;
        int64_t OFF1  = 0;
        int64_t SENS1 = 0;

        T1    = pow(dT, 2) / pow (2, 31);
        OFF1  = 61 * pow((TEMP - 2000), 2) / pow(2, 4);
        SENS1 = 2 * pow((TEMP - 2000), 2);

        if(TEMP < -1500) // if temperature lower than -15 Celsius
        {
            OFF1  = OFF1 + 15 * pow((TEMP + 1500), 2);
            SENS1 = SENS1 + 8 * pow((TEMP + 1500), 2);
        }

        TEMP -= T1;
        OFF -= OFF1;
        SENS -= SENS1;
    }


    Temperature = (float)TEMP / 100;

    P  = ((int64_t)D1 * SENS / pow(2, 21) - OFF) / pow(2, 15);

    Pressure = (float)P / 100;
    tempKelvin = Temperature + 273.15;

    //altitude = (1-pow((Pressure/1013.25),.190284)) * 145366.45;
    altitude = (1-pow((Pressure/1013.25),.190264)) * 44330.76923; //best so far
    //altitude = (1-pow((Pressure/1013.25),.190264)) * 45340.76923;
    // altitude = - (log(Pressure/1013.25) * tempKelvin * 29.263);

    correctedPressure = (Pressure/(exp(-altitude/(tempKelvin * 29.263))));

    altitude = altitude * 3.2808;


}

float CoolSatBaro::getTemp() {
    return Temperature;
}

float CoolSatBaro::getPressure() {
    return Pressure;
}

double CoolSatBaro::getCorrectedPressure() {
    return correctedPressure;
}

float CoolSatBaro::getAltitude() {
    return altitude;
}