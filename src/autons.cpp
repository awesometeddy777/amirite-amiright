#include "autons.h"

void autonSelector(bool allowRunNow = false)
{
    while (true)
    {
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT))
        {
            lastAuton();
            controller.print(0, 0, "AUTON: %s           ", getAutonName(currentAuton));
        }
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_UP))
        {
            nextAuton();
            controller.print(0, 0, "AUTON: %s           ", getAutonName(currentAuton));
        }
        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_Y))
        {
            controller.print(0, 0, "Selected!                                      ");
            if (allowRunNow && !pros::competition::is_connected())
            {
                runAuton();
            }
            break;
        }
        pros::delay(10);
    }
}

void noAuton()
{
}

void left2p7()
{
}

void right2p7()
{
}

void left9()
{
    // driveStraightNew(1000);
    // driveTurn(-90);
    // driveArcRF(90, 500, 127, 800);
    blockBlockToggle = true;
    blockBlock.set_value(blockBlockToggle);

    intake.move(127);
    // // driveStraightNew(990);
    // driveStraightContinuous({{57, 30}, {25, 10}}, 5000);
    // driveTurn(68);
    driveStraightChain({
        {12, 100}, //
        {23, 35},  //
        {3, 50}    //
    });
    driveTurn(-68, 80, 600);
    driveStraightChain({
        {2, 42} //
    });
    delay(100);
    licker.set_value(true);
    delay(300);
    driveStraightChain({
        {-5, 73} //

    });
    driveTurn(18, 80, 700);
    driveStraightChain({
        {-8, 75} //
    });
    driveTurn(63, 80, 800);
    licker.set_value(false);
    delay(300);
    angleChanger.set_value(false);
    driveStraightNew(430, 800);

    intake.move(127);
    blockBlock.set_value(false);
    delay(1200);
    intake.move(-70);
    delay(600);
    blockBlock.set_value(true);
    intake.move(127);
    // intake.move(127);
    driveStraightChain({
        {-10, 90}, //
        {-10, 73},
        {-11, 45} //
    });

    driveTurn(-162, 80, 800);
    licker.set_value(true);
    angleChanger.set_value(true);
    delay(500);
    driveStraightChain({
        {3, 30} //
    });
    delay(1200);
    driveStraightChain({
        {-5, 80} //
    });
    driveTurn(18, 80, 800);
    licker.set_value(false);
    driveStraightChain({
        {4, 60} //
    });
    driveBrake();
    blockBlock.set_value(false);
    intake.move(-127);
    delay(500);
    intake.move(127);
    delay(5000);

    // driveStraightChain({
    //     {-8, 40} //
    // });
    // driveTurn(50, 900);
    // delay(50);
    // driveStraightChain({
    //     {-40, 79} //
    // });
    // driveTurn(20, 500);
    // licker.set_value(false);
    // delay(100);
    // driveStraightNew(752, 700);
    // blockBlock.set_value(false);
    // intake.move(127);
    // delay(500);
    // intake.move(-127);
    // delay(200);
    // intake.move(127);
    // delay(1200);
    // blockBlock.set_value(true);
    // driveStraightChain({
    //     {1, 80}, //
    //     {-6, 50} //
    // });
    // driveTurn(-163, 800);
    // delay(250);
    // licker.set_value(true);
    // delay(200);
    // driveStraightChain({
    //     {2, 70} //
    // });
    // delay(1200);
}
// void left9()
// {
//     // driveStraightNew(1000);
//     // driveTurn(-90);
//     // driveArcRF(90, 500, 127, 800);
//     blockBlockToggle = true;
//     blockBlock.set_value(blockBlockToggle);

//     intake.move(127);
//     // // driveStraightNew(990);
//     // driveStraightContinuous({{57, 30}, {25, 10}}, 5000);
//     // driveTurn(68);
//     driveStraightChain({
//         {12, 100}, //
//         {23, 23},  //
//         {10, 17}   //
//     });
//     driveTurn(-64, 700);
//     driveStraightChain({
//         {2, 40} //
//     });
//     delay(300);
//     licker.set_value(true);
//     delay(300);
//     driveStraightChain({
//         {-15, 25} //
//     });
//     driveTurn(60, 900);
// }

void right9()
{
}
void redSolo()
{
}

void blueSolo()
{
}

//! CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE CHANGEEEEE
autonSelect currentAuton = left9Auton;

void nextAuton()
{
    switch (currentAuton)
    {
    case none:
        currentAuton = left2p7Auton;
        break;
    case left2p7Auton:
        currentAuton = right2p7Auton;
        break;
    case right2p7Auton:
        currentAuton = left9Auton;
        break;
    case left9Auton:
        currentAuton = right9Auton;
        break;
    case right9Auton:
        currentAuton = redSoloAWP;
        break;
    case redSoloAWP:
        currentAuton = blueSoloAWP;
        break;
    case blueSoloAWP:
        currentAuton = none;
        break;
    default:
        currentAuton = none;
        break;
    }
}

void lastAuton()
{
    switch (currentAuton)
    {
    case none:
        currentAuton = blueSoloAWP;
        break;
    case left2p7Auton:
        currentAuton = none;
        break;
    case right2p7Auton:
        currentAuton = left2p7Auton;
        break;
    case left9Auton:
        currentAuton = right2p7Auton;
        break;
    case right9Auton:
        currentAuton = left9Auton;
        break;
    case redSoloAWP:
        currentAuton = right9Auton;
        break;
    case blueSoloAWP:
        currentAuton = redSoloAWP;
        break;
    default:
        currentAuton = none;
        break;
    }
}

void runAuton()
{
    switch (currentAuton)
    {
    case none:
        noAuton();
        break;
    case left2p7Auton:
        left2p7();
        break;
    case right2p7Auton:
        right2p7();
        break;
    case left9Auton:
        left9();
        break;
    case right9Auton:
        right9();
        break;
    case redSoloAWP:
        redSolo();
        break;
    case blueSoloAWP:
        blueSolo();
        break;
    default:
        // do nothing
        break;
    }
}

const char *getAutonName(autonSelect currentA)
{
    switch (currentA)
    {
    case none:
        return "No Auton";
    case left2p7Auton:
        return "Left 2+7";
    case right2p7Auton:
        return "Right 2+7";
    case left9Auton:
        return "Left 9 Hi";
    case right9Auton:
        return "Right 9 Hi";
    case redSoloAWP:
        return "Red Solo AWP";
    case blueSoloAWP:
        return "Blue Solo AWP";
    default:
        return "Unknown";
    }
}
