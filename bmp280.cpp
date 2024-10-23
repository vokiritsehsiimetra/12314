#include "bmp280.h"

BMP280::BMP280(int address) : address(address), i2cHandle(-1) {
}

bool BMP280::begin() {
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio!" << std::endl;
        return false; // Initialization failed
    }

    // Open the I2C bus
    i2cHandle = gpioI2COpen(1, address, 0); // Bus 1, address
    if (i2cHandle < 0) {
        std::cerr << "Failed to open I2C bus: " << i2cHandle << std::endl;
        return false; // Handle initialization failed
    }

    // Read calibration data
    readCalibrationData();
    
    // Set the control register for the sensor
    gpioI2CWriteByteData(i2cHandle, 0xF4, 0x27); // Normal mode, 16x oversampling for temp and pressure
    return true;
}

void BMP280::readCalibrationData() {
    dig_T1 = readRegister16(0x88);
    dig_T2 = readRegister16(0x8A);
    dig_T3 = readRegister16(0x8C);
    dig_P1 = readRegister16(0x8E);
    dig_P2 = readRegister16(0x90);
    dig_P3 = readRegister16(0x92);
    dig_P4 = readRegister16(0x94);
    dig_P5 = readRegister16(0x96);
    dig_P6 = readRegister16(0x98);
    dig_P7 = readRegister16(0x9A);
    dig_P8 = readRegister16(0x9C);
    dig_P9 = readRegister16(0x9E);

    // Debug output for calibration data
    std::cout << "Calibration data:" << std::endl;
    std::cout << "dig_T1: " << dig_T1 << ", dig_T2: " << dig_T2 << ", dig_T3: " << dig_T3 << std::endl;
    std::cout << "dig_P1: " << dig_P1 << ", dig_P2: " << dig_P2 << ", dig_P3: " << dig_P3 << std::endl;
    std::cout << "dig_P4: " << dig_P4 << ", dig_P5: " << dig_P5 << ", dig_P6: " << dig_P6 << std::endl;
    std::cout << "dig_P7: " << dig_P7 << ", dig_P8: " << dig_P8 << ", dig_P9: " << dig_P9 << std::endl;
}

int BMP280::readRegister(uint8_t reg) {
    int result = gpioI2CReadByteData(i2cHandle, reg);
    if (result < 0) {
        std::cerr << "Error reading register: " << (int)reg << std::endl;
    }
    return result;
}

int BMP280::readRegister16(uint8_t reg) {
    uint8_t msb = readRegister(reg);
    uint8_t lsb = readRegister(reg + 1);
    return (msb << 8) | lsb;
}

int BMP280::readRegister24(uint8_t reg) {
    uint8_t msb = readRegister(reg);
    uint8_t lsb = readRegister(reg + 1);
    uint8_t xlsb = readRegister(reg + 2);
    return (msb << 16) | (lsb << 8) | xlsb;
}

int32_t BMP280::compensateTemperature(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - (dig_T1 << 1))) * dig_T2) >> 11);
    int32_t var2 = (((((adc_T >> 4) - dig_T1) * ((adc_T >> 4) - dig_T1)) >> 12) * dig_T3) >> 14);
    return var1 + var2;
}

uint32_t BMP280::compensatePressure(int32_t adc_P) {
    int32_t var1 = (((int64_t)compensateTemperature(adc_P)) >> 1) - 64000;
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
    int32_t adc_T = readRegister24(0xFA) >> 4; // Read the temperature data
    std::cout << "Raw Temperature ADC: " << adc_T << std::endl; // Debug output
    int32_t t = compensateTemperature(adc_T);
    return t / 5120.0;
}

float BMP280::readPressure() {
    int32_t adc_P = readRegister24(0xF7) >> 4; // Read the pressure data
    std::cout << "Raw Pressure ADC: " << adc_P << std::endl; // Debug output
    return compensatePressure(adc_P) / 256.0; // Convert to hPa
}

float BMP280::readAltitude(float seaLevelPressure) {
    float pressure = readPressure();
    return (1.0 - pow(pressure / seaLevelPressure, 0.190294957)) * 44330.0; // Convert to meters
}
