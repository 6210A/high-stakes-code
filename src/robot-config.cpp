#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
triport Expander17 = triport(PORT17);
motor LFDrive = motor(PORT1, ratio18_1, true);
motor LTDrive = motor(PORT3, ratio18_1, false);
motor LBDrive = motor(PORT2, ratio18_1, true);
motor RFDrive = motor(PORT16, ratio18_1, true);
motor RTDrive = motor(PORT13, ratio18_1, false);
motor RBDrive = motor(PORT14, ratio18_1, false);
motor LeftIntake = motor(PORT7, ratio18_1, true);
motor RightIntake = motor(PORT12, ratio18_1, false);
motor Arm = motor(PORT18, ratio18_1, false);
inertial Inertial9 = inertial(PORT9);
rotation OdomForward = rotation(PORT20, false);
rotation OdomSideways = rotation(PORT21, false);
controller Controller1 = controller(primary);
optical Optical = optical(PORT11);
digital_out MogoMech = digital_out(Expander17.F);

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