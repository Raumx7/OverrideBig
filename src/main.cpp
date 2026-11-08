/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       raulmomox                                                 */
/*    Created:      9/2/2026, 1:50:46 PM                                      */
/*    Description:  Override Big Robot competition code                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include <cmath>

#include "vex.h"

using namespace vex;

// Drive constants
constexpr int MAX_DRIVE = 100;
constexpr int DEAD_ZONE = 5;
constexpr int MAX_SPEED = 60;

// Gears constants
constexpr double GEAR_RATIO = 60.0 / 12.0;
constexpr double TARGET_DEG = GEAR_RATIO * 100.0;

// Controller
controller Controller = controller(primary);

// Drivetrain motors left
motor mtr_l1 = motor(PORT10, true);
motor mtr_l2 = motor(PORT9, true);

// Drivetrain motors right
motor mtr_r1 = motor(PORT20);
motor mtr_r2 = motor(PORT19);

// Motor groups
motor_group motors_l = motor_group(mtr_l1, mtr_l2);
motor_group motors_r = motor_group(mtr_r1, mtr_r2);

// Arm motors
motor arm_l = motor(PORT17, true);
motor arm_r = motor(PORT18);

// In-take motor
motor intake = motor(PORT8);

// Claw motors
motor claw_intake = motor(PORT12, true);
motor claw_lift = motor(PORT13, true);

void move_claw()
{

    static bool claw_lifted = false;

    double target_deg = claw_lifted ? 0.0 : TARGET_DEG;

    claw_lift.spinToPosition(
        target_deg,
        rotationUnits::deg,
        MAX_DRIVE,
        velocityUnits::pct,
        false
    );

    claw_lifted = !claw_lifted;

}

int clamp(int val, int min, int max)
{
    if (val > max)
        return max;
    if (val < min)
        return min;
    return val;
}

int main()
{

    arm_l.setBrake(brakeType::hold);
    arm_r.setBrake(brakeType::hold);

    claw_lift.setBrake(brakeType::hold);

    claw_lift.setPosition(0.0, rotationUnits::deg);

    Controller.ButtonX.pressed(move_claw);

    while (true)
    {

        int forward = Controller.Axis3.position();
        int turn = Controller.Axis1.position();

        if (std::abs(forward) < DEAD_ZONE)
            forward = 0;
        if (std::abs(turn) < DEAD_ZONE)
            turn = 0;

        int power_l = clamp(forward + turn, -MAX_DRIVE, MAX_DRIVE);
        int power_r = clamp(forward - turn, -MAX_DRIVE, MAX_DRIVE);

        motors_l.spin(directionType::fwd, power_l, velocityUnits::pct);
        motors_r.spin(directionType::fwd, power_r, velocityUnits::pct);

        if (Controller.ButtonR1.pressing())
        {
            arm_l.spin(directionType::fwd, -MAX_SPEED, velocityUnits::pct);
            arm_r.spin(directionType::fwd, -MAX_SPEED, velocityUnits::pct);
        }

        else if (Controller.ButtonR2.pressing())
        {
            arm_l.spin(directionType::fwd, MAX_SPEED, velocityUnits::pct);
            arm_r.spin(directionType::fwd, MAX_SPEED, velocityUnits::pct);
        }

        else
        {
            arm_l.stop();
            arm_r.stop();
        }

        if (Controller.ButtonA.pressing())
            intake.spin(directionType::fwd, MAX_DRIVE, velocityUnits::pct);

        else if (Controller.ButtonB.pressing())
            intake.spin(directionType::fwd, -MAX_DRIVE, velocityUnits::pct);

        else
            intake.stop();

        if (Controller.ButtonL1.pressing())
            claw_intake.spin(directionType::fwd, MAX_DRIVE, velocityUnits::pct);
        

        else if (Controller.ButtonL2.pressing())
            claw_intake.spin(directionType::fwd, -MAX_DRIVE, velocityUnits::pct);
        

        else
            claw_intake.stop();

        wait(30, timeUnits::msec);
    }
}
