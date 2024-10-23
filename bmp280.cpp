#include "BMP280.h"

BMP280::BMP280(int address) : address(address) {
    fd = wiringPiI2CSetup(address);
}

bool BMP280::begin() {
    if (fd == -1) return false;

    // Read calibration data
    readCalibrationData();
    
    // Set the control register for the sensor
    wiringPiI2CWriteReg8(fd, 0xF4, 0x27); // Normal mode, 16x oversampling for both temp and pressure
    return true;
}

void BMP280::readCalibrationData() {
    dig_T1 = wiringPiI2CReadReg16(fd, 0x88);
    dig_T2 = wiringPiI2CReadReg16(fd, 0x8A);
    dig_T3 = wiringPiI2CReadReg16(fd, 0x8C);
    dig_P1 = wiringPiI2CReadReg16(fd, 0x8E);
    dig_P2 = wiringPiI2CReadReg16(fd, 0x90);
    dig_P3 = wiringPiI2CReadReg16(fd, 0x92);
    dig_P4 = wiringPiI2CReadReg16(fd, 0x94);
    dig_P5 = wiringPiI2CReadReg16(fd, 0x96);
    dig_P6 = wiringPiI2CReadReg16(fd, 0x98);
    dig_P7 = wiringPiI2CReadReg16(fd, 0x9A);
    dig_P8 = wiringPiI2CReadReg16(fd, 0x9C);
    dig_P9 = wiringPiI2CReadReg16(fd, 0x9E);
}

int32_t BMP280::compensateTemperature(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - (dig_T1 << 1))) * dig_T2) >> 11);
    int32_t var2 = (((((adc_T >> 4) - dig_T1) * ((adc_T >> 4) - dig_T1)) >> 12) * dig_T3) >> 14);
    return var1 + var2;
}

uint32_t BMP280::compensatePressure(int32_t adc_P) {
    int32_t var1 = (((int64_t)compensateTemperature(adc_P)) >> 1) - 64000);
    int32_t var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * dig_P6;
    var2 = var2 + ((var1 * dig_P5) << 1);
    var2 = (var2 >> 2) + (dig_P4 << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((dig_P2 * var1) >> 1)) >> 18);
    var1 = ((32768 + var1) * dig_P1) >> 15;

    if (var1 == 0) return 0; // Avoid exception caused by division by zero

    uint32_t p = ((1048576 - adc_P) - (var2 >> 12)) * 3125;
    p = (p < 0x80000000) ? (p << 1) / var1 : (p / var1) >> 1;
    var1 = (dig_P9 * ((p >> 3) * (p >> 3)) >> 13);
    var2 = (dig_P8 * p) >> 13;
    p += (var1 + var2 + dig_P7) >> 4;

    return p;
}

float BMP280::readTemperature() {
    int32_t adc_T = wiringPiI2CReadReg24(fd, 0xFA) >> 4;
    int32_t t = compensateTemperature(adc_T);
    return t / 5120.0;
}

float BMP280::readPressure() {
    int32_t adc_P = wiringPiI2CReadReg24(fd, 0xF7) >> 4;
    return compensatePressure(adc_P) / 256.0; // Convert to hPa
}

float BMP280::readAltitude(float seaLevelPressure) {
    float pressure = readPressure();
    return (1.0 - pow(pressure / seaLevelPressure, 0.190294957)) * 44330.0; // Convert to meters
}