#include "pid.h"
// #include "autons.h"`
#include <iostream>

using namespace pros;
using namespace c;
using namespace std;

double trueTarget = 0;

double hKp;
double hKi;
double hKd;
double error;
double prevError;
double integral;
double derivative;
int time2;
double power;

double hKp2;
double hKi2;
double hKd2;
double error2;
double prevError2;
double integral2;
double derivative2;
double power2;

double hKp3;
double hKi3;
double hKd3;
double error3;
double prevError3;
double integral3;
double derivative3;
double power3;

void setConstants(double kp, double ki, double kd)
{
    hKp = kp;
    hKi = ki;
    hKd = kd;
}

void setConstants2(double kp, double ki, double kd)
{
    hKp2 = kp;
    hKi2 = ki;
    hKd2 = kd;
}

double calcPID(double target, double input, int integralKi, int maxIntegral, bool slewOn = false)
{ // basically tuning i here

    int integral;

    prevError = error;
    error = target - input;

    if (abs(error) < integralKi)
    {
        integral += error;
    }
    else
    {
        integral = 0;
    }

    if (integral >= 0)
    {
        integral = min(integral, maxIntegral); // min means take whichever value is smaller btwn integral and maxI
        // integral = integral until integral is greater than maxI (to keep integral limited to maxI)
    }
    else
    {
        integral = max(integral, -maxIntegral); // same thing but negative max
    }

    derivative = error - prevError;

    power = (hKp * error) + (hKi * integral) + (hKd * derivative);

    return power;
}

double calcPID2(double target, double input, int integralKi, int maxIntegral, bool slewOn = false)
{ // basically tuning i here
    int integral2;
    prevError2 = error2;
    error2 = target - input;

    if (std::abs(error2) < integralKi)
    {
        integral2 += error2;
    }
    else
    {
        integral2 = 0;
    }

    if (integral2 >= 0)
    {
        integral2 = std::min(integral2, maxIntegral); // min means take whichever value is smaller btwn integral and maxI
        // integral = integral until integral is greater than maxI (to keep integral limited to maxI)
    }
    else
    {
        integral2 = std::max(integral2, -maxIntegral); // same thing but negative max
    }

    derivative2 = error2 - prevError2;

    power2 = (hKp * error2) + (hKi * integral2) + (hKd * derivative2);

    return power2;
}

double calcPID3(double target, double input, int integralKi, int maxIntegral, bool slewOn = false)
{ // basically tuning i here
    int integral3;
    prevError3 = error3;
    error3 = target - input;

    if (std::abs(error3) < integralKi)
    {
        integral3 += error3;
    }
    else
    {
        integral3 = 0;
    }

    if (integral3 >= 0)
    {
        integral3 = std::min(integral3, maxIntegral); // min means take whichever value is smaller btwn integral and maxI
        // integral = integral until integral is greater than maxI (to keep integral limited to maxI)
    }
    else
    {
        integral3 = std::max(integral3, -maxIntegral); // same thing but negative max
    }

    derivative3 = error3 - prevError3;

    power3 = (hKp * error3) + (hKi * integral3) + (hKd * derivative3);

    return power3;
}

void driveStraight(int target)
{

    // imu.tare();
    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0; // controller Display Cycle
    time2 = 0;

    if (init_heading > 180)
    {
        init_heading = init_heading - 360;
    }

    int timeout = 30000;
    double x = 0;
    x = double(abs(target));

    // timeout = (0 * pow(x,5)) + (0 * pow(x, 4)) + (0 * pow(x, 3)) + (0 * pow(x, 2)) + (0 * x) + 0; //Tune with Desmos

    resetEncoders();
    while (true)
    {
        if (abs(target - encoderAvg) < 25)
        {
            setConstants(2.5, 0, 0);
        }
        else
        {
            setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        }

        encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        double position = imu.get_heading(); // this is where the units are set to be degrees

        if (position > 180)
        {
            position = position - 360;
        }

        if ((init_heading < 0) && (position > 0))
        {
            if ((position - init_heading) >= 180)
            {
                init_heading = init_heading + 360;
                position = imu.get_heading();
            }
        }
        else if ((init_heading > 0) && (position < 0))
        {
            if ((init_heading - position) >= 180)
            {
                position = imu.get_heading();
            }
        }

        // if(longValues){
        //     setConstants(HEADING_KP2, HEADING_KI2, HEADING_KD2);
        // } else {
        //     setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
        // }
        setConstants(HEADING_KP, HEADING_KI, HEADING_KD);

        heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        if (voltage > 127)
        {
            voltage = 127;
        }
        else if (voltage < -127)
        {
            voltage = -127;
        }
        // errorp = abs(target - encoderAvg);
        heading_error = 0;
        driveVolts((voltage + heading_error), (voltage - heading_error), 0);
        if (abs(target - encoderAvg) <= 2)
            count++;
        if (count >= 8 || time2 > timeout)
        {
            break;
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0)
        {
            controller.print(0, 0, "ERROR: %f           ", float(error));
        }
        else if (time2 % 100 == 0 && time2 % 150 != 0)
        {
            controller.print(1, 0, "imu: %f           ", float(imu.get_heading()));
        }
        else if (time2 % 150 == 0)
        {
            controller.print(2, 0, "Time: %f        ", float(time2));
        }

        delay(10);
        time2 += 10;
    }
    driveBrake();
}

void driveStraightNew(int target, int timeout)
{

    // imu.tare();
    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0; // Controller Display Cycle
    time2 = 0;

    if (init_heading > 180)
    {
        init_heading = init_heading - 360;
    }

    double x = 0;
    x = double(abs(target));

    resetEncoders();
    while (true)
    {
        if (abs(target - encoderAvg) < 25)
        {
            // setConstants(2.5, 0, 0);
            setConstants(0.9, 0, 1.6);
        }
        else
        {
            setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        }

        encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        double position = imu.get_heading(); // this is where the units are set to be degrees

        if (position > 180)
        {
            position = position - 360;
        }

        if ((init_heading < 0) && (position > 0))
        {
            if ((position - init_heading) >= 180)
            {
                init_heading = init_heading + 360;
                position = imu.get_heading();
            }
        }
        else if ((init_heading > 0) && (position < 0))
        {
            if ((init_heading - position) >= 180)
            {
                position = imu.get_heading();
            }
        }

        setConstants(HEADING_KP, HEADING_KI, HEADING_KD);

        heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        if (voltage > 127)
        {
            voltage = 127;
        }
        else if (voltage < -127)
        {
            voltage = -127;
        }
        heading_error = 0;

        driveVolts((voltage + heading_error), (voltage - heading_error), 0);
        if (abs(target - encoderAvg) <= 2)
            count++;
        if (count >= 8 || time2 > timeout)
        {
            break;
        }

        // if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0)
        // {
        //     controller.print(0, 0, "ERROR: %f           ", float(error));
        // }
        // else if (time2 % 100 == 0 && time2 % 150 != 0)
        // {
        //     controller.print(1, 0, "vkp: %f           ", float(hKp));
        // }
        // else if (time2 % 150 == 0)
        // {
        //     controller.print(2, 0, "Time: %f        ", float(time2));
        // }

        delay(10);
        time2 += 10;
    }
    driveBrake();
}

void driveStraightInches(double targetInches)
{
    // === Conversion to motor degrees ===
    double wheelDiameter = 3.25; // change if you use 4" or other wheels
    double wheelCircumference = wheelDiameter * M_PI;
    double targetDegrees = (targetInches / wheelCircumference) * 360.0;

    imu.tare();
    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0; // Controller Display Cycle
    time2 = 0;

    int timeout = 30000;
    if (init_heading > 180)
    {
        init_heading = init_heading - 360;
    }

    resetEncoders();

    while (true)
    {
        encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2;

        if (fabs(targetDegrees - encoderAvg) < 25)
        {
            setConstants(2.5, 0, 0);
        }
        else
        {
            setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        }

        voltage = calcPID(targetDegrees, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        double position = imu.get_heading();
        if (position > 180)
        {
            position = position - 360;
        }

        // Fix wrap-around issues
        if ((init_heading < 0) && (position > 0))
        {
            if ((position - init_heading) >= 180)
            {
                init_heading = init_heading + 360;
                position = imu.get_heading();
            }
        }
        else if ((init_heading > 0) && (position < 0))
        {
            if ((init_heading - position) >= 180)
            {
                position = imu.get_heading();
            }
        }

        setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
        heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        // Clamp voltage
        if (voltage > 127)
            voltage = 127;
        else if (voltage < -127)
            voltage = -127;

        // Apply PID corrections
        driveVolts((voltage + heading_error), (voltage - heading_error), 0);

        if (fabs(targetDegrees - encoderAvg) <= 2)
            count++;
        if (count >= 8 || time2 > timeout)
            break;

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0)
        {
            controller.print(0, 0, "ERROR: %f           ", float(error));
        }
        else if (time2 % 100 == 0 && time2 % 150 != 0)
        {
            controller.print(1, 0, "vkp: %f           ", float(hKp));
        }
        else if (time2 % 150 == 0)
        {
            controller.print(2, 0, "Time: %f        ", float(time2));
        }

        delay(10);
        time2 += 10;
    }
    driveBrake();
}

/*
// void driveStraight2(int target, int speedPct)
// {
//     int timeout = 5000; // ms
//     int count = 0;
//     time2 = 0;

//     double voltage = 0;
//     double encoderAvg = 0;
//     double heading_error = 0;

//     // normalize target heading
//     trueTarget = imu.get_heading();
//     if (trueTarget > 180)
//     {
//         trueTarget -= 360;
//     }

//     setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
//     setConstants2(HEADING_KP, HEADING_KI, HEADING_KD);

//     resetEncoders();
//     encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2.0;

//     while (true)
//     {
//         // update encoder position
//         encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2.0;

//         // adjust PID constants near the target
//         if (fabs(target - encoderAvg) < 25)
//         {
//             setConstants(2.5, 0, 0); // simple proportional near stop
//         }
//         else
//         {
//             setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
//         }

//         // base forward PID (distance)
//         voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

//         // current IMU heading (convert to ±180)
//         double position = imu.get_heading();
//         if (position > 180)
//             position -= 360;

//         // heading correction PID
//         heading_error = calcPID2(trueTarget, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

//         // scale voltage to speed percentage
//         double speedFactor = double(speedPct) / 100.0;
//         if (voltage > 127 * speedFactor)
//             voltage = 127 * speedFactor;
//         if (voltage < -127 * speedFactor)
//             voltage = -127 * speedFactor;

//         // drive with correction
//         driveVolts(int(round(voltage + heading_error)), int(round(voltage - heading_error)), 0);

//         // exit conditions
//         if (fabs(target - encoderAvg) <= 4)
//             count++;
//         if (count >= 8 || time2 > timeout)
//             break;

//         // debug to controller
//         if (time2 % 100 == 0)
//         {
//             controller.print(0, 0, "Err: %.1f HeadErr: %.1f", error, heading_error);
//         }

//         pros::delay(10);
//         time2 += 10;
//     }

//     driveBrake();
// }
*/

void driveStraight2(int target, int speed, int timeout)
{

    bool over = false;
    double voltage;
    double encoderAvg;
    int count = 0;
    double heading_error = 0;
    int cycle = 0; // Controller Display Cycle
    time2 = 0;

    if (trueTarget > 180)
    {
        trueTarget = trueTarget - 360;
    }

    resetEncoders();

    while (true)
    {

        if (abs(target - encoderAvg) < 25)
        {
            setConstants(2.5, 0, 0);
        }
        else
        {
            setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        }

        encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2;

        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        double position = imu.get_heading(); // this is where the units are set to be degrees

        if (position > 180)
        {
            position = position - 360;
        }

        if ((trueTarget < 0) && (position > 0))
        {
            if ((position - trueTarget) >= 180)
            {
                trueTarget = trueTarget + 360;
                position = imu.get_heading();
            }
        }
        else if ((trueTarget > 0) && (position < 0))
        {
            if ((trueTarget - position) >= 180)
            {
                position = imu.get_heading();
            }
        }

        // if(trueTarget > 180) {
        //     trueTarget = (360 - trueTarget);
        // }

        // if(imu.get_heading() < 180) {
        //     heading_error = trueTarget - imu.get_heading();
        // }
        // else {
        //     heading_error = ((360 - imu.get_heading()) - trueTarget);
        // }

        // heading_error = heading_error * HEADING_CORRECTION_KP;
        heading_error = calcPID2(trueTarget, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        if (voltage > 127 * double(speed) / 100.0)
        {
            voltage = 127 * double(speed) / 100.0;
        }
        else if (voltage < -127 * double(speed) / 100.0)
        {
            voltage = -127 * double(speed) / 100.0;
        }

        driveVolts((voltage + heading_error), (voltage - heading_error), 0);
        if (abs(target - encoderAvg) <= 4)
            count++;
        if (count >= 8 || time2 > timeout)
        {
            break;
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0)
        {
            controller.print(0, 0, "ERROR: %f           ", float(error));
        }
        else if (time2 % 100 == 0 && time2 % 150 != 0)
        {
            controller.print(1, 0, "true: %f           ", float(trueTarget));
        }
        else if (time2 % 150 == 0)
        {
            controller.print(2, 0, "Time: %f        ", float(time2));
        }

        delay(10);
        time2 += 10;
        // hi
    }
    driveBrake();
}

void driveStraightC(int targetInches, int speed)
{
    targetInches *= 35.26;
    int target = (int)targetInches;
    bool over = false;
    int timeout = 5000;

    if (target > 0)
    {
        target += 500;
    }
    else
    {
        target -= 500;
    }

    double voltage;
    double encoderAvg;
    double heading_error = 0;
    time2 = 0;

    if (trueTarget > 180)
    {
        trueTarget -= 360;
    }

    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    resetEncoders();

    while (true)
    {
        encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        double position = imu.get_heading();
        if (position > 180)
            position -= 360;

        // Wrap-around logic
        if ((trueTarget < 0) && (position > 0))
        {
            if ((position - trueTarget) >= 180)
            {
                trueTarget += 360;
                position = imu.get_heading();
            }
        }
        else if ((trueTarget > 0) && (position < 0))
        {
            if ((trueTarget - position) >= 180)
            {
                position = imu.get_heading();
            }
        }

        // Heading PID
        setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
        heading_error = calcPID2(trueTarget, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        // Clamp with speed %
        int maxVolt = 127 * speed / 100;
        if (voltage > maxVolt)
            voltage = maxVolt;
        if (voltage < -maxVolt)
            voltage = -maxVolt;

        driveVolts(voltage + heading_error, voltage - heading_error, 0);

        // Exit conditions
        if (target > 0)
        {
            if ((encoderAvg - (target - 500)) > 0)
                over = true;
        }
        else
        {
            if (((target + 500) - encoderAvg) > 0)
                over = true;
        }

        if (over || time2 > timeout)
        {
            break;
        }

        delay(10);
        time2 += 10;
    }
}

void driveStraightChain(std::vector<StraightMove> moves)
{
    for (auto seg : moves)
    {
        driveStraightC(seg.target, seg.speed);
    }
}

void driveTurn(int target, int speed = 100, int timeout = 5000)
{
    trueTarget = target;
    double voltage;
    double position;
    int count = 0;
    time2 = 0;

    setConstants(TURN_KP, TURN_KI, TURN_KD);

    while (true)
    {
        position = imu.get_heading();

        // Convert heading to -180 -> 180
        if (position > 180)
        {
            position -= 360;
        }

        // Find shortest path error (-180 to 180)
        double error = target - position;
        if (error > 180)
            error -= 360;
        if (error < -180)
            error += 360;

        // PID output
        voltage = calcPID(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL);

        // Apply speed cap (like driveStraightC)
        int maxVolt = 127 * speed / 100; // scale by % speed
        if (voltage > maxVolt)
            voltage = maxVolt;
        if (voltage < -maxVolt)
            voltage = -maxVolt;

        driveVolts(voltage, -voltage, 0);

        // Settled condition
        if (fabs(error) <= 0.5)
            count++;
        else
            count = 0; // reset if error jumps back

        if (count >= 20 || time2 > timeout)
        {
            break;
        }

        time2 += 10;
        delay(10);
    }

    driveBrake();
}

void driveArcRF(double theta, double radius, int speed, int timeout)
{
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    const double TRACK_WIDTH = 304.8;
    bool over = false;
    int trueTheta = theta;
    double ltarget = 0;
    double rtarget = 0;
    double ltargetFinal = 0;
    double rtargetFinal = 0;
    double pi = 3.14159265359;
    // if (trueTarget > 180){
    //     trueTarget farc= trueTarget - 360;
    // }
    int count = 0;
    int time = 0;
    double rightcorrect = 0;
    resetEncoders();
    // int timeout = 5000;
    ltargetFinal = double((theta / 360) * 2 * pi * (radius + TRACK_WIDTH)); // * double(2) * pi * double(radius));
    rtargetFinal = double((theta / 360) * 2 * pi * (radius));

    ltarget = double((theta / 360) * 2 * pi * (radius + TRACK_WIDTH)); // * double(2) * pi * double(radius));
    rtarget = double((theta / 360) * 2 * pi * (radius));

    while (true)
    {
        if (trueTarget > 180)
        {
            trueTarget = trueTarget - 360;
        }

        double position = imu.get_heading(); // this is where the units are set to be degrees W

        if (position > 180)
        {
            position = position - 360;
        }
        double encoderAvgR = (right_motor_back.get_position() + right_motor_middle.get_position()) / 2;
        rightcorrect = (encoderAvgR * 360) / (2 * pi * radius);

        if (((trueTarget + rightcorrect) < 0) && (position > 0))
        {
            if ((position - (trueTarget + rightcorrect)) >= 180)
            {
                trueTarget = trueTarget + 360;
                position = imu.get_heading();
            }
        }
        else if (((trueTarget + rightcorrect) > 0) && (position < 0))
        {
            if (((trueTarget + rightcorrect) - position) >= 180)
            {
                position = imu.get_heading();
            }
        }

        double encoderAvgL = left_motor_front.get_position();
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if (voltageL > 127 * double(speed) / 100.0)
        {
            voltageL = 127 * double(speed) / 100.0;
        }
        else if (voltageL < -127 * double(speed) / 100.0)
        {
            voltageL = -127 * double(speed) / 100.0;
        }

        int voltageR = calcPID2(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if (voltageR > 127 * double(speed) / 100.0)
        {
            voltageR = 127 * double(speed) / 100.0;
        }
        else if (voltageR < -127 * double(speed) / 100.0)
        {
            voltageR = -127 * double(speed) / 100.0;
        }

        setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
        int fix = calcPID3((trueTarget + rightcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);

        driveVolts((voltageL + fix), (voltageR - fix), 0);

        // if (theta > 0){
        //     if ((encoderAvgR - rtargetFinal) > 0){
        //         over = true;
        //     }
        // } else {
        //     if ((rtargetFinal - encoderAvgR) > 0){
        //         over = true;
        //     }
        // }

        if (theta > 0)
        {
            if (abs((trueTarget - position)) > trueTheta)
            {
                over = true;
            }
        }
        else
        {
            if (abs((position - trueTarget)) > -trueTheta)
            {
                over = true;
            }
        }

        if (over || time > timeout)
        {
            trueTarget += trueTheta;
            break;
        }

        time += 10;
        delay(10);
    }
}

void driveArcLF1(double theta, double radius, int speed, int timeout)
{
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    const double TRACK_WIDTH = 304.8;
    int trueTheta = theta;
    double ltarget = 0;
    double rtarget = 0;
    double ltargetFinal = 0;
    double rtargetFinal = 0;
    double pi = 3.14159265359;
    bool over = false;
    int count = 0;
    int time = 0;
    resetEncoders();

    // int timeout = 5000;
    ltargetFinal = double((theta / 360) * 2 * pi * radius); // * double(2) * pi * double(radius));
    rtargetFinal = double((theta / 360) * 2 * pi * (radius + TRACK_WIDTH));
    if (theta > 0)
    {
        theta = theta + 45;
    }
    else
    {
        theta = theta - 45;
    }
    ltarget = double((theta / 360) * 2 * pi * radius); // * double(2) * pi * double(radius));
    rtarget = double((theta / 360) * 2 * pi * (radius + TRACK_WIDTH));
    while (true)
    {

        double encoderAvgL = left_motor_front.get_position();
        double encoderAvgR = (right_motor_back.get_position() + right_motor_middle.get_position()) / 2;
        double leftcorrect = -(encoderAvgL * 360) / (2 * pi * radius);

        if (trueTarget > 180)
        {
            trueTarget = trueTarget - 360;
        }

        double position = imu.get_heading(); // this is where the units are set to be degrees W

        if (position > 180)
        {
            position = position - 360;
        }

        if (((trueTarget + leftcorrect) < 0) && (position > 0))
        {
            if ((position - (trueTarget + leftcorrect)) >= 180)
            {
                leftcorrect = leftcorrect + 360;
                position = imu.get_heading();
            }
        }
        else if (((trueTarget + leftcorrect) > 0) && (position < 0))
        {
            if (((trueTarget + leftcorrect) - position) >= 180)
            {
                position = imu.get_heading();
            }
        }

        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        // if(voltageL > 70){ //set left limit
        //     voltageL = 70;
        // } else if (voltageL < -70){
        //     voltageL = -70;
        // }

        if (voltageL > 127 * double(speed) / 100.0)
        {
            voltageL = 127 * double(speed) / 100.0;
        }
        else if (voltageL < -127 * double(speed) / 100.0)
        {
            voltageL = -127 * double(speed) / 100.0;
        }

        int voltageR = calcPID2(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        if (voltageR > 127 * double(speed) / 100.0)
        {
            voltageR = 127 * double(speed) / 100.0;
        }
        else if (voltageR < -127 * double(speed) / 100.0)
        {
            voltageR = -127 * double(speed) / 100.0;
        }

        setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
        int fix = calcPID3((trueTarget + leftcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);

        driveVolts((voltageL + fix), (voltageR - fix), 0);

        // if (theta > 0){
        //     if ((encoderAvgL - ltargetFinal) > 0){
        //         over = true;
        //     }
        // } else {
        //     if ((ltargetFinal - encoderAvgL) > 0){
        //         over = true;
        //     }
        // }
        if (theta > 0)
        {
            if (abs((trueTarget - position)) > trueTheta)
            {
                over = true;
            }
        }
        else
        {
            if (abs((position - trueTarget)) > -trueTheta)
            {
                over = true;
            }
        }

        if (over || time > timeout)
        {
            trueTarget -= trueTheta;
            break;
        }

        time += 10;
        delay(10);
    }
}

void driveArcRFa(double theta, double radius, int speed, int timeout)
{
    // Constants
    const double pi = 3.14159265359;
    const double wheelDiameter = 82.55; // change to your wheel diameter in mm
    const double wheelCircumference = pi * wheelDiameter;
    const double trackWidth = 304.8; // distance between left/right wheels in mm
    radius *= 25.4;                  // convert in → mm

    resetEncoders();
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);

    // Compute distance each wheel should travel (linear distance)
    double leftDistance = 2 * pi * (radius + trackWidth / 2.0) * (theta / 360.0);
    double rightDistance = 2 * pi * (radius - trackWidth / 2.0) * (theta / 360.0);

    // Convert distance to encoder degrees
    double ltarget = (leftDistance / wheelCircumference) * 360.0;
    double rtarget = (rightDistance / wheelCircumference) * 360.0;

    int elapsedTime = 0;
    bool finished = false;

    while (!finished && elapsedTime < timeout)
    {
        // Read encoder positions
        double leftPos = (left_motor_front.get_position() + left_motor_middle.get_position() + left_motor_back.get_position()) / 3.0;
        double rightPos = (right_motor_front.get_position() + right_motor_middle.get_position() + right_motor_back.get_position()) / 3.0;

        // PID for left/right
        int voltageL = calcPID(ltarget, leftPos, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        int voltageR = calcPID2(rtarget, rightPos, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        // Limit voltage by speed
        voltageL = std::clamp(voltageL, -127 * speed / 100, 127 * speed / 100);
        voltageR = std::clamp(voltageR, -127 * speed / 100, 127 * speed / 100);

        // IMU heading correction
        double targetHeading = theta; // relative target
        double heading = imu.get_heading();
        double headingError = targetHeading - heading;
        while (headingError > 180)
            headingError -= 360;
        while (headingError < -180)
            headingError += 360;

        setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
        int fix = calcPID3(headingError, 0, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);

        // Drive motors with heading correction
        driveVolts(voltageL + fix, voltageR - fix, 0);

        // Check if both wheels reached target
        if (abs(leftPos - ltarget) < 5 && abs(rightPos - rtarget) < 5)
        { // 5 deg tolerance
            finished = true;
        }

        elapsedTime += 10;
        delay(10);
    }

    // Stop motors at the end
    driveBrake();
}
void driveArcRFnew(double theta, double radius, int speed, int timeout)
{
    const double pi = 3.14159265359;
    const double wheelDiameter = 82.55;
    const double wheelCircumference = pi * wheelDiameter;
    const double trackWidth = 304.8;

    radius *= 25.4; // convert inches → mm
    resetEncoders();

    // PID constants
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    const double headingKP = ARC_HEADING_KP;
    const double headingKI = ARC_HEADING_KI;
    const double headingKD = ARC_HEADING_KD;

    // Compute distances each wheel should travel
    double leftDistance = 2 * pi * (radius + trackWidth / 2.0) * (theta / 360.0);
    double rightDistance = 2 * pi * (radius - trackWidth / 2.0) * (theta / 360.0);

    double ltarget = (leftDistance / wheelCircumference) * 360.0;
    double rtarget = (rightDistance / wheelCircumference) * 360.0;

    double startHeading = imu.get_heading();
    int elapsedTime = 0;
    bool finished = false;

    while (!finished && elapsedTime < timeout)
    {
        // Read average motor positions
        double leftPos = (left_motor_front.get_position() + left_motor_middle.get_position() + left_motor_back.get_position()) / 3.0;
        double rightPos = (right_motor_front.get_position() + right_motor_middle.get_position() + right_motor_back.get_position()) / 3.0;

        // Base PID for wheel positions
        int voltageL = calcPID(ltarget, leftPos, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        int voltageR = calcPID2(rtarget, rightPos, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        // Heading correction (relative)
        double targetHeading = startHeading + theta;
        double heading = imu.get_heading();
        double headingError = targetHeading - heading;

        while (headingError > 180)
            headingError -= 360;
        while (headingError < -180)
            headingError += 360;

        int fix = calcPID3(headingError, 0, headingKI, ARC_HEADING_MAX_INTEGRAL);

        // Apply heading correction
        int finalL = voltageL + fix;
        int finalR = voltageR - fix;

        // Clamp final voltages
        finalL = std::clamp(finalL, -127 * speed / 100, 127 * speed / 100);
        finalR = std::clamp(finalR, -127 * speed / 100, 127 * speed / 100);

        driveVolts(finalL, finalR, 0);

        // Check if within tolerance
        if (abs(leftPos - ltarget) < 5 && abs(rightPos - rtarget) < 5)
            finished = true;

        elapsedTime += 10;
        delay(10);
    }

    driveBrake();
}

void driveArcLF(double theta, double radius, int speed, int timeout)
{
    // Constants
    const double pi = 3.14159265359;
    const double wheelDiameter = 82.55; // 3.25 in in mm
    const double wheelCircumference = pi * wheelDiameter;
    const double trackWidth = 304.8; // distance between left/right wheels in mm

    resetEncoders();
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);

    // Compute distance for counterclockwise arc
    double leftDistance = 2 * pi * (radius - trackWidth / 2.0) * (theta / 360.0);  // left wheel goes "smaller radius"
    double rightDistance = 2 * pi * (radius + trackWidth / 2.0) * (theta / 360.0); // right wheel goes "larger radius"

    // Convert distances to encoder degrees
    double ltarget = (leftDistance / wheelCircumference) * 360.0;
    double rtarget = (rightDistance / wheelCircumference) * 360.0;

    int elapsedTime = 0;
    bool finished = false;

    while (!finished && elapsedTime < timeout)
    {
        // Average encoder positions
        double leftPos = (left_motor_front.get_position() + left_motor_middle.get_position() + left_motor_back.get_position()) / 3.0;
        double rightPos = (right_motor_front.get_position() + right_motor_middle.get_position() + right_motor_back.get_position()) / 3.0;

        // PID for left/right wheels
        int voltageL = calcPID(ltarget, leftPos, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        int voltageR = calcPID2(rtarget, rightPos, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        // Limit voltage by speed
        voltageL = std::clamp(voltageL, -127 * speed / 100, 127 * speed / 100);
        voltageR = std::clamp(voltageR, -127 * speed / 100, 127 * speed / 100);

        // IMU heading correction
        double targetHeading = theta; // relative target
        double heading = imu.get_heading();
        double headingError = targetHeading - heading;
        while (headingError > 180)
            headingError -= 360;
        while (headingError < -180)
            headingError += 360;

        setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
        int fix = calcPID3(headingError, 0, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);

        // Drive motors with heading correction
        driveVolts(voltageL + fix, voltageR - fix, 0);

        // Check if both wheels reached target
        if (abs(leftPos - ltarget) < 5 && abs(rightPos - rtarget) < 5)
        {
            finished = true;
        }

        elapsedTime += 10;
        delay(10);
    }

    // Stop motors at the end
    driveVolts(0, 0, 0);
}

/*
#include "api.h"
#include "main.h"
#include "pid.h"
#include "robot.h"
#include "drive.h"

using namespace pros;
using namespace std;

double trueTarget = 0;

double hKp = 0;
double hKi = 0;
double hKd = 0;
double error = 0;
double prevError = 0;
double derivative = 0;
int time2 = 0;
double power = 0;

double hKp2 = 0;
double hKi2 = 0;
double hKd2 = 0;
double error2 = 0;
double prevError2 = 0;
double derivative2 = 0;
double power2 = 0;

void setConstants(double kp, double ki, double kd)
{
    hKp = kp;
    hKi = ki;
    hKd = kd;
}

void setConstants2(double kp, double ki, double kd)
{
    hKp2 = kp;
    hKi2 = ki;
    hKd2 = kd;
}

// Fixed: use persistent integral accumulators and avoid shadowing/uninitialized variables
double calcPID(double target, double input, int integralKi, int maxIntegral, bool slewOn = false)
{
    static double integral_accum = 0.0;

    prevError = error;
    error = target - input;

    if (std::abs(error) < integralKi)
    {
        integral_accum += error;
    }
    else
    {
        integral_accum = 0.0;
    }

    if (integral_accum >= 0.0)
    {
        integral_accum = std::min(integral_accum, double(maxIntegral));
    }
    else
    {
        integral_accum = std::max(integral_accum, -double(maxIntegral));
    }

    derivative = error - prevError;

    power = (hKp * error) + (hKi * integral_accum) + (hKd * derivative);

    return power;
}

double calcPID2(double target, double input, int integralKi, int maxIntegral, bool slewOn = false)
{
    static double integral_accum2 = 0.0;

    prevError2 = error2;
    error2 = target - input;

    if (std::abs(error2) < integralKi)
    {
        integral_accum2 += error2;
    }
    else
    {
        integral_accum2 = 0.0;
    }

    if (integral_accum2 >= 0.0)
    {
        integral_accum2 = std::min(integral_accum2, double(maxIntegral));
    }
    else
    {
        integral_accum2 = std::max(integral_accum2, -double(maxIntegral));
    }

    derivative2 = error2 - prevError2;

    // Fixed: use second PID constants
    power2 = (hKp2 * error2) + (hKi2 * integral_accum2) + (hKd2 * derivative2);

    return power2;
}

void driveStraight(int target)
{
    imu.tare();
    double voltage = 0;
    double encoderAvg = 0;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    time2 = 0;

    if (init_heading > 180)
    {
        init_heading = init_heading - 360;
    }

    int timeout = 30000;
    double x = double(abs(target));

    resetEncoders();
    // initialize encoderAvg before use
    encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2.0;

    while (true)
    {
        // recompute encoderAvg at start of loop
        encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2.0;

        if (abs(target - encoderAvg) < 25)
        {
            setConstants(2.5, 0, 0);
        }
        else
        {
            setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        }

        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, int(STRAIGHT_MAX_INTEGRAL));

        double position = imu.get_heading();
        if (position > 180)
        {
            position = position - 360;
        }

        if ((init_heading < 0) && (position > 0))
        {
            if ((position - init_heading) >= 180)
            {
                init_heading = init_heading + 360;
                position = imu.get_heading();
            }
        }
        else if ((init_heading > 0) && (position < 0))
        {
            if ((init_heading - position) >= 180)
            {
                position = imu.get_heading();
            }
        }

        setConstants(HEADING_KP, HEADING_KI, HEADING_KD);

        heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        if (voltage > 127)
        {
            voltage = 127;
        }
        else if (voltage < -127)
        {
            voltage = -127;
        }

        // IMPORTANT: do not zero heading_error here — use it for correction
        driveVolts(int(round(voltage + heading_error)), int(round(voltage - heading_error)), 0);

        if (abs(target - encoderAvg) <= 2)
            count++;
        if (count >= 8 || time2 > timeout)
        {
            break;
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0)
        {
            controller.print(0, 0, "ERROR: %f           ", float(error));
        }
        else if (time2 % 100 == 0 && time2 % 150 != 0)
        {
            controller.print(1, 0, "imu: %f           ", float(imu.get_heading()));
        }
        else if (time2 % 150 == 0)
        {
            controller.print(2, 0, "Time: %f        ", float(time2));
        }

        pros::delay(10);
        time2 += 10;
    }
    driveBrake();
}

void driveStraight2(int target, int speed)
{
    int timeout = 5000;

    double voltage = 0;
    double encoderAvg = 0;
    int count = 0;
    double heading_error = 0;
    time2 = 0;

    if (trueTarget > 180)
    {
        trueTarget = trueTarget - 360;
    }

    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);

    resetEncoders();
    encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2.0;

    while (true)
    {
        encoderAvg = (left_motor_front.get_position() + right_motor_front.get_position()) / 2.0;

        if (abs(target - encoderAvg) < 25)
        {
            setConstants(2.5, 0, 0);
        }
        else
        {
            setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        }

        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, int(STRAIGHT_MAX_INTEGRAL));

        double position = imu.get_heading();
        if (position > 180)
        {
            position = position - 360;
        }

        if ((trueTarget < 0) && (position > 0))
        {
            if ((position - trueTarget) >= 180)
            {
                trueTarget = trueTarget + 360;
                position = imu.get_heading();
            }
        }
        else if ((trueTarget > 0) && (position < 0))
        {
            if ((trueTarget - position) >= 180)
            {
                position = imu.get_heading();
            }
        }

        setConstants(HEADING_KP, HEADING_KI, HEADING_KD);

        heading_error = calcPID2(trueTarget, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        double speedFactor = double(speed) / 100.0;
        if (voltage > 127 * speedFactor)
        {
            voltage = 127 * speedFactor;
        }
        else if (voltage < -127 * speedFactor)
        {
            voltage = -127 * speedFactor;
        }

        driveVolts(int(round(voltage + heading_error)), int(round(voltage - heading_error)), 0);
        if (abs(target - encoderAvg) <= 4)
            count++;
        if (count >= 8 || time2 > timeout)
        {
            break;
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0)
        {
            controller.print(0, 0, "ERROR: %f           ", float(error));
        }
        else if (time2 % 100 == 0 && time2 % 150 != 0)
        {
            controller.print(1, 0, "true: %f           ", float(trueTarget));
        }
        else if (time2 % 150 == 0)
        {
            controller.print(2, 0, "Time: %f        ", float(time2));
        }

        pros::delay(10);
        time2 += 10;
    }
    driveBrake();
}*/