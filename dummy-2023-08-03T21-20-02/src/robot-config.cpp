#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
 
// VEXcode device constructors
motor l1 = motor(PORT11, ratio6_1, true);
motor l2 = motor(PORT12, ratio6_1, true);
motor l3 = motor(PORT13, ratio6_1, true);
motor r1 = motor(PORT18, ratio6_1, false);
motor r2 = motor(PORT19, ratio6_1, false);
motor r3 = motor(PORT20, ratio6_1, false);
inertial inert = inertial(PORT10);

rotation rra = rotation(PORT17, true);
controller Controller1 = controller(primary);
motor Intake = motor(PORT21, ratio18_1, false);
motor cata = motor(PORT1, ratio18_1, false);
limit LimitSwitchA = limit(Brain.ThreeWirePort.A);
digital_out arms = digital_out(Brain.ThreeWirePort.F);
digital_out wingR = digital_out(Brain.ThreeWirePort.D);
digital_out wingL = digital_out(Brain.ThreeWirePort.B);
digital_out pto = digital_out(Brain.ThreeWirePort.A);
distance di = distance(PORT2);
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