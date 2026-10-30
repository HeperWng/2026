#pragma once
#include "main.h"

// Sensors
extern pros::IMU gyro;

// Drive
extern void gyroTurn(double targetAngle, double kP, double kI, double kD, int timeout = 0);
extern void gyroDrive(double targetAngle, double targetDistance, double turnkP, double drivekP, bool reverse = false, int timeout = 0);

// Rollers
#define DIGITAL_UP pros::E_CONTROLLER_DIGITAL_L1
#define DIGITAL_DOWN pros::E_CONTROLLER_DIGITAL_L2  
#define DIGITAL_R1 pros::E_CONTROLLER_DIGITAL_R1
#define DIGITAL_R2 pros::E_CONTROLLER_DIGITAL_R2
#define DIGITAL_L1 pros::E_CONTROLLER_DIGITAL_UP
#define DIGITAL_L2 pros::E_CONTROLLER_DIGITAL_DOWN
extern pros::Motor intake;
extern void intakeControl();
extern void rollersInit();
extern void rollerSpinIn();
extern void rollerSpinOut();
extern void rollerSpinStop();
extern void rollerArmUp();
extern void rollerArmDownPos();
extern void rollerArmWait(int timeout);
extern double rollerArmPosition();