#ifndef BMP280_H
#define BMP280_H

#include <wiringPiI2C.h>
#include <cstdint>
#include <cmath>

class BMP280 {
public:
    BMP280(int address = 0x76);
    bool begin();
    float readTemperature();
    float readPressure();
    float readAltitude(float seaLevelPressure = 101325.0);

private:
    int fd;
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
};

#endif // BMP280_H
