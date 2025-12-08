#include "vex.h"
#include "Robot_Config.h"

using namespace vex;

controller Controller1 = controller();
brain Brain = brain();
motor Left11 = motor(PORT11, ratio6_1, true); //Front Left
motor Left12 = motor(PORT12, ratio6_1, true);  //Middle Left
motor Left13 = motor(PORT13, ratio6_1, true); //Back Left Wheel
motor Right18 = motor(PORT18, ratio6_1, true); //Front Right
motor Right19 = motor(PORT19, ratio6_1, true); //Middle Right
motor Right20 = motor(PORT20, ratio6_1, true); //Back Right Wheel

inertial Inertial4 = inertial(PORT4);
motor Stick15 = motor(PORT15, ratio18_1, false);
motor FIP16 = motor(PORT16, ratio6_1, true);
digital_out Tongue = digital_out(Brain.ThreeWirePort.E);
digital_out Gate = digital_out(Brain.ThreeWirePort.D);
digital_out Finger = digital_out(Brain.ThreeWirePort.F);