#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <cmath>

#include "pointing.hpp"

// Reads a number from user input after a prompt in the terminal.
bool readNumber(const std::string &prompt, double &value)
{
    std::cout << prompt << ": ";

    std::string line;
    if (!std::getline(std::cin, line))
    {
        std::cerr << "Error: Unable to read input.\n";
        return false;
    }

    std::istringstream input(line);
    std::string extra;

    if (!(input >> value) ||
        (input >> extra) ||
        !std::isfinite(value))
    {
        std::cerr << "Error: Enter a valid finite number.\n";
        return false;
    }

    return true;
}

// Reads GPS Position by prompting the user to input individual numbers.
bool readGPSPosition(const std::string &name, GPSPosition &pos)
{
    std::cout << "\nEnter " << name << " coordinates:\n";

    if (!readNumber("Latitude (degrees)", pos.latitude))
        return false;

    if (pos.latitude < -90.0 || pos.latitude > 90.0)
    {
        std::cerr << "Error: Latitude must be between -90 and 90.\n";
        return false;
    }

    if (!readNumber("Longitude (degrees)", pos.longitude))
        return false;

    if (pos.longitude < -180.0 || pos.longitude > 180.0)
    {
        std::cerr << "Error: Longitude must be between -180 and 180.\n";
        return false;
    }

    if (!readNumber("Altitude (meters)", pos.altitude))
        return false;

    return true;
}

int main()
{
    GPSPosition rocket;
    GPSPosition groundStation;

    // User inputs gsPosition first then rPosition
    if (!readGPSPosition("Ground Station", groundStation) ||
        !readGPSPosition("Rocket", rocket))
    {
        return 1;
    }

    auto result = calculatePointingAngles(rocket, groundStation);

    if (result.has_value())
    {
        std::cout << "\n=== Pointing Results ===\n";
        std::cout << std::fixed << std::setprecision(4);

        std::cout << "Azimuth: " << result->azimuth << " degrees\n";
        std::cout << "Elevation: " << result->elevation << " degrees\n";
    }
    else
    {
        std::cout << "Invalid GPS coordinates or undefined pointing direction.\n";
        return 1;
    }

    return 0;
}