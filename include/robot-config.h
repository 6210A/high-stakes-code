using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LFDrive;
extern motor LTDrive;
extern motor LBDrive;
extern motor RFDrive;
extern motor RTDrive;
extern motor RBDrive;
extern motor LeftIntake;
extern motor RightIntake;
extern motor Arm;
extern inertial Inertial9;
extern rotation OdomForward;
extern rotation OdomSideways;
extern controller Controller1;
extern optical Optical;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );