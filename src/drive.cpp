#include "main.h"

// Defines
#define deg_to_rad (M_PI / 180.0)
#define rad_to_deg (180.0 / M_PI)
#define integral_max 10000.0
#define integral_min -10000.0
#define max_drive_speed 8000
#define min_drive_speed 2000
#define acceptable_error 1
#define time_in_range 250
#define inch_to_cm 2.54
#define gear_ratio (36 / 60)

// Drivetrain Motorgroup definitions
pros::MotorGroup leftDrive({-16, -17}, pros::v5::MotorGears::blue);
pros::MotorGroup rightDrive({14, 15}, pros::v5::MotorGears::blue);

// Drivetrain settings
lemlib::Drivetrain drivetrain(&leftDrive,                  // Left motors
                              &rightDrive,                 // Right motors
                              15.4,                        // 15.4 inch track width(change later)
                              lemlib::Omniwheel::OLD_275,  // Using 2.75" omni wheel
                              450,                         // Drivetrain rpm is 450(check later)
                              2);                          // HorizontalDrift is 2(for now)

// Angle that bounds input angle between [ -180, 180] degrees
double constrainAngle(double angle) {
    return atan2(sin(angle * deg_to_rad), cos(angle * deg_to_rad)) * rad_to_deg;
}

void driveControl() {
    const int tipkP = 35;
    double forwardSpeed = controller.get_analog(ANALOG_LEFT_Y) * 12000 / 127;
    double rightSpeed = controller.get_analog(ANALOG_RIGHT_X) * 12000 / 127;

    leftDrive.move_voltage(forwardSpeed + tipSpeed + rightSpeed);
    rightDrive.move_voltage(forwardSpeed + tipSpeed - rightSpeed);
}

void gyroTurn(double targetAngle, double kP, double kI, double kD, int timeout) {
    drivetrain.leftMotors->set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
    drivetrain.rightMotors->set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE)
    double error = 0;
    double lastError = 0;
    double derivative = 0;
    double integral = 0;
    double currentAngle = 0;
    double outputSpeed = 0;

    int32_t startTime = pros::millis();
    int32_t timeSinceOutOfBounds = 0;
    int32_t timeInbounds = 0;

    while (timeInbounds < time_in_range) {
        // Constrains the gyroscope value to be within the bounds of [ -180, 180 ]
        currentAngle = constrainAngle(gyro.get_heading());
        error = constrainAngle(targetAngle - currentAngle);

        // Mittigates integral wiindup by only adding when error is small
        if (abs(error) < 10)
            integral += error;

        // Clamps the bounds of the integral to a maximum value
        integral = std::clamp(integral, integral_min, integral_max);

        // If we pass the setpoint in either direction then reset the integral to 0
        if (lastError > 0 && error < 0 || (lastError < 0 && error > 0))
            integral = 0;

        derivative = error - lastError;

        outputSpeed = (error * 12000 / 360 * kP) + (integral * kI) + (derivative * kD);
        drivetrain.leftMotors->move_velocity(outputSpeed);
        drivetrain.rightMotors->move_velocity(-outputSpeed);

        lastError = error;

        if (pros::millis() - startTime > timeout && timeout != 0)
            break;

        if (abs(error) > acceptable_error) {
            timeSinceOutOfBounds = pros::millis();
        }

        timeInbounds = pros::millis() - timeSinceOutOfBounds;
        pros::delay(15);
    }

    drivetrain.leftMotors->brake();
    drivetrain.rightMotors->brake();
}

// Makes the robot drive a fixed distance(cm) while maintaining a fixed angle(degrees)
void gyroDrive(double targetAngle, double targetDistance, double turnkP, double drivekP, bool reverse, int timeout = 0) {
    drivetrain.leftMotors->set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
    drivetrain.rightMotors->set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
    drivetrain.leftMotors->set_encoder_units_all(pros::v5::MotorUnits::degrees);
    drivetrain.rightMotors->set_encoder_units_all(pros::v5::MotorUnits::degrees);

    uint32_t startTime = pros::millis();

    const double wheelDiameter = 2.75 * 2.54;                // inches to cm
    const double wheelCircumference = M_PI * wheelDiameter;  // finding wheel circumference in cm

    double currentAngle = 0;
    double angleError = 0;
    double motorRotations = 0;
    double wheelRotations = 0;
    double currentDistance = 0;
    double distanceError = 0;
    double turningSpeed = 0;
    double drivingSpeed = 0;

    while (currentDistance <= targetDistance) {
        if (pros::millis() > timeout && timeout != 0)
            break;
        // Constrains the gyroscope value to be within the bounds of [ -180, 180 ]
        currentAngle = constrainAngle(gyro.get_heading());
        angleError = constrainAngle(targetAngle - currentAngle);
        turningSpeed = angleError * turnkP * 12000 / 360;

        std::vector<double> leftMotorPositions = drivetrain.leftMotors->get_position_all();
        std::vector<double> rightMotorPositions = drivetrain.rightMotors->get_position_all();

        motorRotations = (fabs(leftMotorPositions[0]) +
                          fabs(leftMotorPositions[1]) +
                          fabs(rightMotorPositions[0]) +
                          fabs(rightMotorPositions[1])) /
                         4;

        wheelRotations = motorRotations * gear_ratio;
        currentDistance = wheelRotations * wheelCircumference;
        distanceError = targetDistance - currentDistance;
        drivingSpeed = std::clamp(distanceError * drivekP, min_drive_speed, max_drive_speed);
        drivingSpeed = reverse ? -drivingSpeed : drivingSpeed;

        drivetrain.leftMotors->move_voltage(drivingSpeed + turningSpeed);
        drivetrain.rightMotors->move_voltage(drivingSpeed - turningSpeed);

        if ((pros::millis() - startTime > timeout) && timeout) {
            break;

            pros::delay(10);
        }
    }

    drivetrain.leftMotors->brake();
    drivetrain.rightMotors->brake();
}
