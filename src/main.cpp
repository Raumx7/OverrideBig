/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       raulmomox                                                 */
/*    Created:      11/11/2026, 1:54:00 PM                                    */
/*    Description:  Override Big Robot autonomous code                        */
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
competition Competition;

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

// Drivetrain
drivetrain chasis = drivetrain(
    motors_l,           // motors left
    motors_r,           // motors right
    82.5,               // wheelTravel
    355.0,              // trackWidth
    310.0,              // wheelBase
    distanceUnits::mm,  // measured in mm
    0.6                 // gearRatio
);

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

template<typename T>
T clamp(T val, T min, T max)
{
    if (val > max)
        return max;
    if (val < min)
        return min;
    return val;
}

void driveDistanceMm(double distanceMm) {
    chasis.driveFor(directionType::fwd, distanceMm, distanceUnits::mm);
}

void driveDistanceCm(double distanceCm) {
    driveDistanceMm(distanceCm * 10.0);
}

void turnByDegrees(double angleDeg) {
    chasis.turnFor(turnType::right, angleDeg, rotationUnits::deg);
}

void autonomous() {
    chasis.setDriveVelocity(100, velocityUnits::pct);
    chasis.setTurnVelocity(100, velocityUnits::pct);

    driveDistanceCm(30.0);
    turnByDegrees(90.0);
}

void pre_auton() {
    arm_l.setBrake(brakeType::hold);
    arm_r.setBrake(brakeType::hold);
    claw_lift.setBrake(brakeType::hold);
    claw_lift.setPosition(0.0, rotationUnits::deg);
    Controller.ButtonX.pressed(move_claw);
}

void usercontrol() {
    while (true) {
        int forwardInput = Controller.Axis3.position();
        int turnInput = Controller.Axis1.position();

        if (std::abs(forwardInput) < DEAD_ZONE)
            forwardInput = 0;
        if (std::abs(turnInput) < DEAD_ZONE)
            turnInput = 0;

        int powerLeft = clamp(forwardInput + turnInput, -MAX_DRIVE, MAX_DRIVE);
        int powerRight = clamp(forwardInput - turnInput, -MAX_DRIVE, MAX_DRIVE);
        motors_l.spin(directionType::fwd, powerLeft, velocityUnits::pct);
        motors_r.spin(directionType::fwd, powerRight, velocityUnits::pct);

        if (Controller.ButtonR1.pressing()) {
            arm_l.spin(directionType::fwd, -MAX_SPEED, velocityUnits::pct);
            arm_r.spin(directionType::fwd, -MAX_SPEED, velocityUnits::pct);
        } else if (Controller.ButtonR2.pressing()) {
            arm_l.spin(directionType::fwd, MAX_SPEED, velocityUnits::pct);
            arm_r.spin(directionType::fwd, MAX_SPEED, velocityUnits::pct);
        } else {
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

int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
    pre_auton();

    while (true)
        wait(100, timeUnits::msec);
}
