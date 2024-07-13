using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor RFDrive;
extern motor RMDrive;
extern motor RBDrive;
extern motor LFDrive;
extern motor LMDrive;
extern motor LBDrive;
extern inertial Inertial14;
extern motor Claw;
extern rotation OdomX;
extern rotation OdomY;
extern digital_out ClawFlip;
extern digital_out BottomClaw;
extern digital_out TopClaw;
extern digital_out MogoMech;
extern motor Arm1;
extern motor Arm2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );