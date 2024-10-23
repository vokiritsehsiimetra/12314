#include <iostream>
#include <unistd.h>  // For sleep function
#include "BMP280.h"

int main() {
    BMP280 bmp;

    // Initialize Pigpio library
    if (!bmp.begin()) {
        std::cerr << "Failed to initialize BMP280 sensor!" << std::endl;
        return 1;
    }

    while (true) {
        // Read temperature
        float temperature = bmp.readTemperature();
        // Read pressure
        float pressure = bmp.readPressure();
        // Read altitude (assuming sea level pressure is 101325 Pa)
        float altitude = bmp.readAltitude(101325.0);

        // Print the readings
        std::cout << "Temperature: " << temperature << " °C" << std::endl;
        std::cout << "Pressure: " << pressure << " hPa" << std::endl;
        std::cout << "Altitude: " << altitude << " m" << std::endl;
        
        // Wait for 1 second before the next reading
        sleep(1);
    }

    return 0;
}
