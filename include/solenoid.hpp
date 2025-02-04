#include "main.h"
#pragma once

class Solenoid{
public:
    Solenoid(std::int32_t SolenoidPort);
    void openSole(void);
    void closeSole(void);

private:
    pros::ADIDigitalOut solenoid;
    const int OPEN_POSITION = 1;
    const int CLOSED_POSITION = 0;

};