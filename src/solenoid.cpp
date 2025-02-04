#include "main.h"
#include "solenoid.hpp"
#ifndef SOLENOID_CPP
#define SOLENOID_CPP

Solenoid::Solenoid(std::int32_t SolenoidPort) : solenoid(SolenoidPort){}

void Solenoid::openSole() {
    solenoid.set_value(1);
}

void Solenoid::closeSole(){
    solenoid.set_value(0);
}

#endif