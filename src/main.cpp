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
constexpr double TARGET_DEG = GEAR_RATIO * 100.0;

// State variables for Callbacks
constexpr double CLAW_DOWN_TOLERANCE = TARGET_DEG * 0.1;
volatile bool intake_mode = false;

// Brain & Controller
brain Brain;
controller Controller = controller(primary);

// Drivetrain motors left
motor mtr_l1 = motor(PORT10, true);
motor mtr_l2 = motor(PORT9, true);

// Drivetrain motors right
motor mtr_r1 = motor(PORT20, false);
motor mtr_r2 = motor(PORT19, false);

// Motor groups
motor_group motors_l = motor_group(mtr_l1, mtr_l2);
motor_group motors_r = motor_group(mtr_r1, mtr_r2);

// Arm motors
motor arm_l = motor(PORT17, true);
motor arm_r = motor(PORT18, false);
motor_group arm = motor_group(arm_l, arm_r);

// Intake & Claw motors
motor intake = motor(PORT8, false);
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
        false // 'false' para no bloquear el hilo y permitir conducir mientras se mueve
    );
}

// Callback ButtonX: sube o baja la garra
void move_claw() {
    if (claw_is_down()) {
        intake_mode = false; // Apaga el modo intake si subes la garra
        claw_to(TARGET_DEG);
    } else {
        claw_to(0.0);
    }
}

// Callback ButtonY: activa/desactiva el modo intake automático y baja/mantiene la garra abajo
void toggle_intake() {
    intake_mode = !intake_mode;
    if (intake_mode) {
        claw_to(0.0); // Baja la garra y la mantiene en 0° con brakeType::hold
    }
}

int clamp(int val, int min_val, int max_val) {
    if (val > max_val) return max_val;
    if (val < min_val) return min_val;
    return val;
}

void pre_auton() {
    // Configuración inicial de frenos y posición cero
    arm.setStopping(brakeType::hold);
    claw_lift.setStopping(brakeType::hold);
    claw_lift.setPosition(0.0, rotationUnits::deg);
}

void autonomous() {
    // Agrega aquí tu rutina autónoma
}

void usercontrol() {
    // Registro de eventos de botones
    Controller.ButtonX.pressed(move_claw);
    Controller.ButtonY.pressed(toggle_intake);

    while (true) {
        // --- CHASIS (Arcade Drive) ---
        int forward = Controller.Axis3.position();
        int turn = Controller.Axis1.position();

        if (std::abs(forward) < DEAD_ZONE) forward = 0;
        if (std::abs(turn) < DEAD_ZONE) turn = 0;

        int power_l = clamp(forward + turn, -MAX_DRIVE, MAX_DRIVE);
        int power_r = clamp(forward - turn, -MAX_DRIVE, MAX_DRIVE);

        motors_l.spin(directionType::fwd, power_l, velocityUnits::pct);
        motors_r.spin(directionType::fwd, power_r, velocityUnits::pct);

        // --- BRAZO (R1 / R2) ---
        if (Controller.ButtonR1.pressing()) {
            arm.spin(directionType::fwd, -MAX_SPEED, velocityUnits::pct);
        } else if (Controller.ButtonR2.pressing()) {
            arm.spin(directionType::fwd, MAX_SPEED, velocityUnits::pct);
        } else {
            arm.stop();
        }

        // --- INTAKE PRINCIPAL (A mete, B saca, o modo automático Y) ---
        if (Controller.ButtonA.pressing()) {
            intake.spin(directionType::fwd, MAX_DRIVE, velocityUnits::pct);
        } else if (Controller.ButtonB.pressing()) {
            intake_mode = false; // Cancela el modo automático si sacas pieza manualmente
            intake.spin(directionType::fwd, -MAX_DRIVE, velocityUnits::pct);
        } else if (intake_mode) {
            intake.spin(directionType::fwd, MAX_DRIVE, velocityUnits::pct);
        } else {
            intake.stop();
        }

        // --- RODILLOS DE GARRA (L1 agarra, L2 suelta, o sigue al modo automático) ---
        if (Controller.ButtonL1.pressing()) {
            claw_intake.spin(directionType::fwd, MAX_DRIVE, velocityUnits::pct);
        } else if (Controller.ButtonL2.pressing()) {
            claw_intake.spin(directionType::fwd, -MAX_DRIVE, velocityUnits::pct);
        } else if (intake_mode) {
            claw_intake.spin(directionType::fwd, MAX_DRIVE, velocityUnits::pct);
        } else {
            claw_intake.stop();
        }

        wait(20, timeUnits::msec);
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
