#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LBackDrive = motor(PORT15, ratio6_1, true);
motor LTopDrive = motor(PORT14, ratio6_1, false);
motor LBottomDrive = motor(PORT16, ratio6_1, true);
motor RTopDrive = motor(PORT12, ratio6_1, true);
motor RBottomDrive = motor(PORT13, ratio6_1, false);
motor IntakeFull = motor(PORT4, ratio6_1, false);
motor IntakeHalf = motor(PORT3, ratio18_1, false);
motor Arm = motor(PORT1, ratio18_1, true);
inertial Inertial8 = inertial(PORT7);
rotation OdomForward = rotation(PORT20, false);
rotation OdomSideways = rotation(PORT21, false);
controller Controller1 = controller(primary);
optical Optical = optical(PORT5);
digital_out MogoMech = digital_out(Brain.ThreeWirePort.A);
digital_out Doinker = digital_out(Brain.ThreeWirePort.B);
digital_out IntakeLift = digital_out(Brain.ThreeWirePort.C);
rotation Rotation2 = rotation(PORT2, true);
motor RBackDrive = motor(PORT11, ratio6_1, false);

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