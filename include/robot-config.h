using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LBackDrive;
extern motor LTopDrive;
extern motor LBottomDrive;
extern motor RTopDrive;
extern motor RBottomDrive;
extern motor IntakeFull;
extern motor IntakeHalf;
extern motor Arm;
extern inertial Inertial8;
extern rotation OdomForward;
extern rotation OdomSideways;
extern controller Controller1;
extern optical Optical;
extern digital_out MogoMech;
extern digital_out Doinker;
extern digital_out IntakeLift;
extern rotation Rotation2;
extern motor RBackDrive;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );