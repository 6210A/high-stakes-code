#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor RFDrive = motor(PORT18, ratio6_1, false);
motor RHalfW = motor(PORT16, ratio6_1, false);
motor RBDrive = motor(PORT17, ratio6_1, false);
motor LFDrive = motor(PORT12, ratio6_1, true);
motor LHalfW = motor(PORT14, ratio6_1, true);
motor LBDrive = motor(PORT13, ratio6_1, true);
inertial Inertial15 = inertial(PORT15);
digital_out MogoMech = digital_out(Brain.ThreeWirePort.F);
optical Optical = optical(PORT19);
digital_out SortingMech = digital_out(Brain.ThreeWirePort.G);
motor RightPTOMotor = motor(PORT20, ratio6_1, false);
motor LeftPTOMotor = motor(PORT11, ratio6_1, true);
digital_out PTO = digital_out(Brain.ThreeWirePort.H);
motor LeftArm = motor(PORT1, ratio18_1, true);
motor RightArm = motor(PORT9, ratio18_1, false);
digital_out ClawPivot = digital_out(Brain.ThreeWirePort.E);
rotation IntakeRotation = rotation(PORT2, false);
digital_out IntakeLift = digital_out(Brain.ThreeWirePort.D);

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