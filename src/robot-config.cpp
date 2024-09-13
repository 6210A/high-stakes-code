#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor RFDrive = motor(PORT1, ratio6_1, false);
motor RHalfW = motor(PORT2, ratio6_1, true);
motor RBDrive = motor(PORT3, ratio6_1, false);
motor LFDrive = motor(PORT5, ratio6_1, true);
motor LHalfW = motor(PORT6, ratio6_1, false);
motor LBDrive = motor(PORT7, ratio6_1, true);
inertial Inertial14 = inertial(PORT14);
rotation OdomX = rotation(PORT8, false);
rotation OdomY = rotation(PORT9, false);
digital_out MogoMech = digital_out(Brain.ThreeWirePort.A);
optical Optical18 = optical(PORT18);
digital_out SortingMech = digital_out(Brain.ThreeWirePort.B);
motor RightPTOMotor = motor(PORT12, ratio18_1, false);
motor LeftPTOMotor = motor(PORT13, ratio18_1, true);
digital_out PTO1 = digital_out(Brain.ThreeWirePort.C);
digital_out PTO2 = digital_out(Brain.ThreeWirePort.D);

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