#ifndef AUTONS
#define AUTONS

#include "main.h"
#include "robot.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include <iostream>
#include "map"
#include "api.h"
#include "pid.h"
#include "drive.h"

// ----- Defines & Variables ----- //
enum autonSelect
{
    none,
    left2p7Auton,
    right2p7Auton,
    left9Auton,
    right9Auton,
    redSoloAWP,
    blueSoloAWP
};
extern autonSelect currentAuton;

// ----- Functions ----- //
extern void autonSelector(bool allowRunNow);
extern void noAuton();
extern void left2p7();
extern void right2p7();
extern void left9();
extern void right9();
extern void redSolo();
extern void blueSolo();

extern void nextAuton();
extern void lastAuton();
extern void runAuton();
extern const char *getAutonName(autonSelect currentA);

#endif