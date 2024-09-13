using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor RFDrive;
extern motor RHalfW;
extern motor RBDrive;
extern motor LFDrive;
extern motor LHalfW;
extern motor LBDrive;
extern inertial Inertial14;
extern rotation OdomX;
extern rotation OdomY;
extern digital_out MogoMech;
extern optical Optical18;
extern digital_out SortingMech;
extern motor RightPTOMotor;
extern motor LeftPTOMotor;
extern digital_out PTO1;
extern digital_out PTO2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );