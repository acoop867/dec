#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
 
// VEXcode device constructors
motor l1 = motor(PORT11, ratio6_1, false);
motor l2 = motor(PORT12, ratio6_1, true);
motor l3 = motor(PORT13, ratio6_1, true);
motor r1 = motor(PORT10, ratio6_1, false);
motor r2 = motor(PORT19, ratio6_1, true);
motor r3 = motor(PORT20, ratio6_1, true);
inertial Inertial4 = inertial(PORT10);
rotation lr = rotation(PORT11, false);
rotation rr = rotation(PORT21, true);
controller Controller1 = controller(primary);
motor Intake = motor(PORT7, ratio18_1, false);
motor cata = motor(PORT10, ratio18_1, false);
limit LimitSwitchA = limit(Brain.ThreeWirePort.A);
digital_out arms = digital_out(Brain.ThreeWirePort.F);
digital_out wingR = digital_out(Brain.ThreeWirePort.E);
digital_out wingL = digital_out(Brain.ThreeWirePort.G);

line lines = line(Brain.ThreeWirePort.E);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}