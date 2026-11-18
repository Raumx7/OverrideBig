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

competition Competition;

// Drive constants
constexpr int MAX_DRIVE = 100;
constexpr int DEAD_ZONE = 5;
constexpr int MAX_SPEED = 60;

// Gears constants
constexpr double GEAR_RATIO = 60.0 / 12.0;
constexpr double TARGET_DEG = GEAR_RATIO * 95.0;

// State variables for Callbacks
constexpr double CLAW_DOWN_TOLERANCE = TARGET_DEG * 0.1;

// Control the intake and claw returning states
bool intake_mode = false;
bool lift_returning = false;

// Brain
brain Brain;

// Controller
controller Controller = controller(primary);

// Drivetrain motors left
motor mtr_l1 = motor(PORT10, true);
motor mtr_l2 = motor(PORT9, true);

// Drivetrain motors right
motor mtr_r1 = motor(PORT20, false);
motor mtr_r2 = motor(PORT19, false);

// Lift motors
motor lift_l = motor(PORT17, false);
motor lift_r = motor(PORT18, true);

// Motor groups
motor_group motors_l = motor_group(mtr_l1, mtr_l2);
motor_group motors_r = motor_group(mtr_r1, mtr_r2);

// Lift motor group
motor_group lift = motor_group(lift_l, lift_r);

// In-take motor
motor intake = motor(PORT8, false);

// Claw motors
motor claw_intake = motor(PORT12, true);
motor claw_lift = motor(PORT13, true);

bool claw_is_down() {
    return claw_lift.position(rotationUnits::deg) <= CLAW_DOWN_TOLERANCE;
}

void claw_to(double target_deg) {
    claw_lift.spinToPosition(
        target_deg,
        rotationUnits::deg,
        MAX_DRIVE,
        velocityUnits::pct,
        false
    );
}

void move_claw() {
    claw_to(claw_is_down() ? TARGET_DEG : 0.0);
}

// Callback ButtonY: toggle automatic intake mode
void toggle_intake() {
    intake_mode = !intake_mode;

    if (intake_mode) {
        if (!claw_is_down()) {
            claw_to(0.0); 
        }
        lift.spinToPosition(0.0, rotationUnits::deg, MAX_SPEED, velocityUnits::pct, false);
        lift_returning = true;
    }
}

int clamp(int val) {
    if (val > MAX_DRIVE) return MAX_DRIVE;
    if (val < -MAX_DRIVE) return -MAX_DRIVE;
    return val;
}

void pre_auton() {
    lift.setStopping(brakeType::hold);

    claw_lift.setBrake(brakeType::hold);
    claw_lift.setPosition(0.0, rotationUnits::deg);

    // The arm's power-on position is its starting position (0 deg)
    lift.setPosition(0.0, rotationUnits::deg);
}

void autonomous() {
    // Agrega aquí tu rutina autónoma
}

void usercontrol() {
    // Claw down or up (0° | 90°)
    Controller.ButtonX.pressed(move_claw);

    // Intake mode (ButtonY toggles it): claw down, intake/claw_intake running and lift dowm
    Controller.ButtonY.pressed(toggle_intake);

    while (true) {
        int forward = Controller.Axis3.position();
        int turn = Controller.Axis1.position();

        if (std::abs(forward) < DEAD_ZONE)
            forward = 0;
        if (std::abs(turn) < DEAD_ZONE)
            turn = 0;

        int power_l = clamp(forward + turn);
        int power_r = clamp(forward - turn);

        motors_l.spin(directionType::fwd, power_l, velocityUnits::pct);
        motors_r.spin(directionType::fwd, power_r, velocityUnits::pct);

        // Pressing R1 or R2 cancels the automatic lift return.
        if (Controller.ButtonR1.pressing())
        {
            lift_returning = false;
            lift.spin(directionType::fwd, MAX_SPEED, velocityUnits::pct);
        }

        else if (Controller.ButtonR2.pressing())
        {
            lift_returning = false;
            lift.spin(directionType::fwd, -MAX_SPEED, velocityUnits::pct);
        }

        // Don't stop the arm while it is returning, or the return would be cancelled
        else if (!lift_returning)
            lift.stop();

        // Intake: A in, B out. Buttons override intake mode while held.
        if (Controller.ButtonA.pressing())
            intake.spin(directionType::fwd, MAX_DRIVE, velocityUnits::pct);

        else if (Controller.ButtonB.pressing())
            intake.spin(directionType::fwd, -MAX_DRIVE, velocityUnits::pct);

        else if (intake_mode)
            intake.spin(directionType::fwd, MAX_DRIVE, velocityUnits::pct);

        else
            intake.stop();

        // Claw: L1 grab, L2 release. Buttons override intake mode while held.
        if (Controller.ButtonL1.pressing())
            claw_intake.spin(directionType::fwd, MAX_DRIVE, velocityUnits::pct);

        else if (Controller.ButtonL2.pressing())
            claw_intake.spin(directionType::fwd, -MAX_DRIVE, velocityUnits::pct);

        else if (intake_mode)
            claw_intake.spin(directionType::fwd, MAX_DRIVE, velocityUnits::pct);

        else
            claw_intake.stop();

        wait(30, timeUnits::msec);
    }
}

int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    pre_auton();

    while (true) {
        wait(100, timeUnits::msec);
    }
}
