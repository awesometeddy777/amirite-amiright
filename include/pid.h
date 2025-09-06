#ifndef PIDH
#define PIDH

#include "api.h"
#include "main.h"
#include "okapi/api.hpp"
#include "pros/api_legacy.h"
#include "robot.h"
#include "drive.h"

// ----- Defines & Variables ----- //
#define HEADING_KP 1 // 5.25//8.75 //15/////////15
#define HEADING_KI 0 // 0.125//0.115
#define HEADING_KD 1 // 38 //105 //70 //100 //180///////////////400

#define HEADING_INTEGRAL_KI 0
#define HEADING_MAX_INTEGRAL 0

#define STRAIGHT_KP 0.192 // 1 //0.7
#define STRAIGHT_KI 0     // 0.001 //0.1
#define STRAIGHT_KD 1.6   // 1 //8 //change

#define STRAIGHT_INTEGRAL_KI 40
#define STRAIGHT_MAX_INTEGRAL 14.5

#define TURN_KP 2.1  // 5.25//8.75
#define TURN_KI 0    // 0.125//0.115
#define TURN_KD 17.5 // 38 //105 //70

#define TURN_INTEGRAL_KI 30
#define TURN_MAX_INTEGRAL 25

#define ARC_HEADING_KP 1
#define ARC_HEADING_KI 0
#define ARC_HEADING_KD 1

#define ARC_HEADING_INTEGRAL_KI 0
#define ARC_HEADING_MAX_INTEGRAL 0

extern double error;

// ----- Functions ----- //
struct StraightMove
{
    int target; // encoder degrees
    int speed;  // % speed
};

extern void setConstants(double kp, double ki, double kd);
extern void setConstants2(double kp, double ki, double kd);
extern double calcPID(double target, double input, int integralKi, int maxIntegral, bool slewOn);
extern double calcPID2(double target, double input, int integralKi, int maxIntegral, bool slewOn);
extern double calcPID3(double target, double input, int integralKi, int maxIntegral, bool slewOn);
extern void driveStraight(int target);
extern void driveStraightNew(int target, int timeout);
extern void driveStraight2(int target, int speed, int timeout);
extern void driveStraightInches(double targetInches);
extern void driveStraightC(int target, int speed);
extern void driveStraightChain(std::vector<StraightMove> moves);
extern void driveTurn(int target, int speed, int timeout);
extern void driveArcRF(double theta, double radius, int speed, int timeout);
extern void driveArcLF(double theta, double radius, int speed, int timeout);

#endif