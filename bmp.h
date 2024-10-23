#ifndef BMP280_H
#define BMP280_H

#include <cstdint>
#include <cmath>
#include <pigpio.h>

class BMP280 {
public:
    BMP280(int address = 0x76);
    bool begin();
    float readTemperature();
    float readPressure();
    float readAltitude(float seaLevelPressure = 101325.0);

private:
    int address;

    // Calibration data
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;

    void readCalibrationData();
    int32_t compensateTemperature(int32_t adc_T);
    uint32_t compensatePressure(int32_t adc_P);
    int readRegister(uint8_t reg);
    int readRegister16(uint8_t reg);
    int readRegister24(uint8_t reg);
};

#endif // BMP280_H
