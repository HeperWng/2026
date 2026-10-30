#ifndef ROLLERS_HPP
#define ROLLERS_HPP

#include "api.h"

// Call once from initialize(). Robot must start with the roller arm UP.
void rollersInit();

// Call every loop in opcontrol().
void rollersControl(pros::Controller& master);

// Roller spin (port 9)
void rollerSpinIn();
void rollerSpinOut();
void rollerSpinStop();

// Roller arm (port 10)
void rollerArmUp();
void rollerArmDownPos();
void rollerArmWait(int timeout = 1500);  // block until arm reaches target (for auto)
double rollerArmPosition();              // current arm position in degrees

#endif
