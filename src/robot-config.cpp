#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor RFDrive = motor(PORT1, ratio6_1, false);
motor RMDrive = motor(PORT2, ratio6_1, true);
motor RBDrive = motor(PORT3, ratio6_1, false);
motor LFDrive = motor(PORT5, ratio6_1, true);
motor LMDrive = motor(PORT6, ratio6_1, false);
motor LBDrive = motor(PORT7, ratio6_1, true);
inertial Inertial20 = inertial(PORT20);
motor Claw = motor(PORT10, ratio18_1, false);
rotation OdomX = rotation(PORT8, false);
rotation OdomY = rotation(PORT9, false);
motor ArmMotorA = motor(PORT11, ratio18_1, false);
motor ArmMotorB = motor(PORT12, ratio18_1, true);
motor_group Arm = motor_group(ArmMotorA, ArmMotorB);
digital_out ClawFlip = digital_out(Brain.ThreeWirePort.A);
digital_out ClawA = digital_out(Brain.ThreeWirePort.B);
digital_out ClawB = digital_out(Brain.ThreeWirePort.C);

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