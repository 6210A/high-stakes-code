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
extern inertial Inertial8;
extern rotation OdomForward;
extern rotation OdomSideways;
extern controller Controller1;
extern optical Optical;
extern triport Expander17;
extern digital_out MogoMech;
extern digital_out Doinker;
extern digital_out HangMech;
extern digital_out IntakeLift;
extern rotation Rotation16;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );