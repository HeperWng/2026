#include "rollers.hpp"
#include "main.h"

#define ROLLER_SPIN_PORT   9    // spins the roller
#define ROLLER_ARM_PORT    10   // raises / lowers the roller

// Arm positions in motor degrees, relative to where it sits at power-on.
// TUNE: jog the arm with the arrows, read the value off the brain screen.
#define ROLLER_ARM_UP_DEG    0.0
#define ROLLER_ARM_DOWN_DEG  200.0
#define ROLLER_ARM_SPEED_RPM 100

void intakeControl() {
    double intakeSpeed = controller.get_digital(DIGITAL_R1)   ? 12000
                         : controller.get_digital(DIGITAL_R2) ? -12000
                                                              : 0;
    double intakeSpeed = rollerspin.get_digital(DIGITAL_L1)   ? 12000
                         : rollerspin.get_digital(DIGITAL_L2) ? -12000
                                                              : 0;
    double intakeSpeed = rollerarm.get_digital(DIGITAL_UP)   ? 12000
                         : rollerarm.get_digital(DIGITAL_DOWN) ? -12000
                                                              : 0;
}

void intakeControl() {
    double intakeSpeed = rollerspin.get_digital(DIGITAL_L1)   ? 12000
                         : rollerspin.get_digital(DIGITAL_L2) ? -12000
                                                              : 0;
    
}

static pros::Motor rollerSpin(ROLLER_SPIN_PORT, pros::v5::MotorGears::green);
static pros::Motor rollerArm(ROLLER_ARM_PORT, pros::v5::MotorGears::green);  // match your cartridge

static bool rollerArmDown = false;

void rollersInit() {
    rollerSpin.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rollerArm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);  // holds position when idle
    rollerArm.set_encoder_units(pros::v5::MotorUnits::degrees);
    rollerArm.tare_position();  // arm must be UP at power-on
    rollerArmDown = false;
}

static void setRollerSpin(int mv) { rollerSpin.move_voltage(mv); }
void rollerSpinIn()   { setRollerSpin(12000); }
void rollerSpinOut()  { setRollerSpin(-12000); }
void rollerSpinStop() { setRollerSpin(0); }

static void rollerArmGoTo(bool down) {
    rollerArmDown = down;
    rollerArm.move_absolute(down ? ROLLER_ARM_DOWN_DEG : ROLLER_ARM_UP_DEG, ROLLER_ARM_SPEED_RPM);
}
void rollerArmUp()      { rollerArmGoTo(false); }
void rollerArmDownPos() { rollerArmGoTo(true); }

void rollerArmWait(int timeout) {
    uint32_t start = pros::millis();
    double target = rollerArmDown ? ROLLER_ARM_DOWN_DEG : ROLLER_ARM_UP_DEG;
    while (fabs(rollerArm.get_position() - target) > 5 && (int)(pros::millis() - start) < timeout)
        pros::delay(10);
}

double rollerArmPosition() { return rollerArm.get_position(); }




