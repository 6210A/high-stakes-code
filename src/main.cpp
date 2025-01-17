// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LBackDrive           motor         15
// LTopDrive            motor         14
// LBottomDrive         motor         16
// RTopDrive            motor         12
// RBottomDrive         motor         13
// IntakeFull           motor         4
// IntakeHalf           motor         3
// Arm                  motor         1
// Inertial8            inertial      8
// OdomForward          rotation      20
// OdomSideways         rotation      21
// Controller1          controller
// Optical              optical       5
// MogoMech             digital_out   A
// Doinker              digital_out   B
// IntakeLift           digital_out   C
// Rotation2            rotation      2
// RBackDrive           motor         11
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "vex.h"
#include <iostream>
#include <string>

using namespace vex;
competition Competition;
Func func;
/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/

Drive chassis(

    // Pick your drive setup from the list below:
    // ZERO_TRACKER_NO_ODOM
    // ZERO_TRACKER_ODOM
    // TANK_ONE_FORWARD_ENCODER
    // TANK_ONE_FORWARD_ROTATION
    // TANK_ONE_SIDEWAYS_ENCODER
    // TANK_ONE_SIDEWAYS_ROTATION
    // TANK_TWO_ENCODER
    // TANK_TWO_ROTATION
    // HOLONOMIC_TWO_ENCODER
    // HOLONOMIC_TWO_ROTATION
    //
    // Write it here:
    ZERO_TRACKER_NO_ODOM,

    // Add the names of your Drive motors into the motor groups below, separated
    // by commas, i.e. motor_group(Motor1,Motor2,Motor3). You will input
    // whatever motor names you chose when you configured your robot using the
    // sidebar configurer, they don't have to be "Motor1" and "Motor2".

    // Left Motors:
    motor_group(LTopDrive, LBottomDrive, LBackDrive),

    // Right Motors:
    motor_group(RTopDrive, RBottomDrive, RBackDrive),

    // Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e.
    // "PORT1", not simply "1"):
    PORT7,

    // Input your wheel diameter. (4" omnis are actually closer to 4.125"):
    3.25,

    // External ratio, must be in decimal, in the format of input teeth/output
    // teeth. If your motor has an 84-tooth gear and your wheel has a 60-tooth
    // gear, this value will be 1.4. If the motor drives the wheel directly,
    // this value is 1:
    0.75,

    // Gyro scale, this is what your gyro reads when you spin the robot 360
    // degrees. For most cases 360 will do fine here, but this scale factor can
    // be very helpful when precision is necessary.
    360,

    /*---------------------------------------------------------------------------*/
    /*                                  PAUSE! */
    /*                                                                           */
    /*  The rest of the drive constructor is for robots using POSITION TRACKING.
     */
    /*  If you are not using position tracking, leave the rest of the values as
     */
    /*  they are. */
    /*---------------------------------------------------------------------------*/

    // If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD
    // TRACKER CENTER DISTANCE.

    // FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is
    // only necessary for holonomic drives, otherwise this section can be left
    // alone. LF:      //RF:
    PORT1, -PORT2,

    // LB:      //RB:
    PORT3, -PORT4,

    // If you are using position tracking, this is the Forward Tracker port (the
    // tracker which runs parallel to the direction of the chassis). If this is
    // a rotation sensor, enter it in "PORT1" format, inputting the port below.
    // If this is an encoder, enter the port as an integer. Triport A will be a
    // "1", Triport B will be a "2", etc.
    PORT20,

    // Input the Forward Tracker diameter (reverse it to make the direction
    // switch):
    2,

    // Input Forward Tracker center distance (a positive distance corresponds to
    // a tracker on the right side of the robot, negative is left.) For a zero
    // tracker tank drive with odom, put the positive distance from the center
    // of the robot to the right side of the drive. This distance is in inches:
    -1.25,

    // Input the Sideways Tracker Port, following the same steps as the Forward
    // Tracker Port:
    PORT21,

    // Sideways tracker diameter (reverse to make the direction switch):
    -2,

    // Sideways tracker center distance (positive distance is behind the center
    // of the robot, negative is in front):
    -0.75

);
bool ispreauto;
int current_auton_selection = 5;
bool auto_started = false;

float armGoal = 1;
bool manualArm = false;

bool driveHold = false;
int driveTorque = 100;
int slowestDrive;
int fastestDrive;
int leftSpeed = 0;
int rightSpeed = 0;
int axis1;
int axis2;
int axis3;
int axis4;

double hookLocation;
bool intakeRunning;
int originalIntakeSpeed;

bool ringDetected;
bool redDetected;
bool blueDetected;

float gyro1;
int msecClock;

std::string sortingColor = "red";

void sleep(int sleepmsec) { task::sleep(sleepmsec); }

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  ispreauto = true;
  default_constants();
  IntakeFull.spin(fwd);
  IntakeHalf.spin(fwd);
  Arm.spin(fwd);
  Arm.setVelocity(-70, pct);
  sleep(100);
  while (Arm.velocity(pct) < -2) {
    sleep(5);
  }
  Arm.setVelocity(0, pct);
  Arm.setPosition(0, deg);
  Rotation2.setPosition(0, deg);
  MogoMech = false;
  ispreauto = false;

  while (!auto_started) {
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "Battery Percentage:");
    Brain.Screen.printAt(5, 40, "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 60, "Heading:");
    Brain.Screen.printAt(5, 80, "%f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 100, "Selected Auton:");
    switch (current_auton_selection) {
    case 0:
      Brain.Screen.printAt(5, 120, "Red Back No WP");
      break;
    case 1:
      Brain.Screen.printAt(5, 120, "Blue Back No WP");
      break;
    case 2:
      Brain.Screen.printAt(5, 120, "Red Goal Rush");
      break;
    case 3:
      Brain.Screen.printAt(5, 120, "Blue Goal Rush");
      break;
    case 4:
      Brain.Screen.printAt(5, 120, "Red Back WP");
      break;
    case 5:
      Brain.Screen.printAt(5, 120, "Blue Back WP");
      break;
    case 6:
      Brain.Screen.printAt(5, 120, "Red Front No WP");
      break;
    case 7:
      Brain.Screen.printAt(5, 120, "Blue Front No WP");
      break;
    case 8:
      Brain.Screen.printAt(5, 120, "Skills Auton");
      break;
    }
    if (Brain.Screen.pressing()) {
      while (Brain.Screen.pressing()) {
      }
      current_auton_selection++;
    } else if (current_auton_selection == 9) {
      current_auton_selection = 0;
    }
    task::sleep(10);
  }
}

int controllerScreenTask() {
  Controller1.Screen.clearScreen();
  while (1) {
    sleep(50);

    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("G: %3.2f", Inertial8.rotation());
    Controller1.Screen.setCursor(1, 13);
    Controller1.Screen.print("A: %1.0f", func.armState);
    if (sortingColor == "red") {
      Controller1.Screen.print("B in");
    } else if (sortingColor == "blue") {
      Controller1.Screen.print("R in");
    }
    // Controller1.Screen.setCursor(1, 16);
    // Controller1.Screen.print("A%d", autonRunning);
    // Controller1.Screen.setCursor(1, 19);
    // Controller1.Screen.print("R%d", ringDetected);

    // Controller1.Screen.setCursor(2, 1);
    // Controller1.Screen.print("%1.0f ",
    //                          RightArm.temperature(fahrenheit) / 10 - 5);
    // Controller1.Screen.setCursor(2, 3);
    // Controller1.Screen.print("%1.0f ",
    //                          LeftArm.temperature(fahrenheit) / 10 - 5);
    // Controller1.Screen.setCursor(2, 6);
    // Controller1.Screen.print("State %1.0f", armState);
    // Controller1.Screen.setCursor(2, 15);
    // Controller1.Screen.print("%1.0f", autonNumber);

    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("%1.0f ",
                             LTopDrive.temperature(fahrenheit) / 10 - 5);

    Controller1.Screen.setCursor(3, 3);
    Controller1.Screen.print("%1.0f ",
                             RTopDrive.temperature(fahrenheit) / 10 - 5);

    Controller1.Screen.setCursor(3, 5);
    Controller1.Screen.print("%1.0f ",
                             LBottomDrive.temperature(fahrenheit) / 10 - 5);

    Controller1.Screen.setCursor(3, 7);
    Controller1.Screen.print("%1.0f ",
                             RBottomDrive.temperature(fahrenheit) / 10 - 5);

    Controller1.Screen.setCursor(3, 9);
    Controller1.Screen.print("%1.0f ",
                             LBackDrive.temperature(fahrenheit) / 10 - 5);

    Controller1.Screen.setCursor(3, 11);
    Controller1.Screen.print("%1.0f ",
                             RBackDrive.temperature(fahrenheit) / 10 - 5);

    Controller1.Screen.setCursor(3, 15);
    Controller1.Screen.print("%1.0f ",
                             IntakeFull.temperature(fahrenheit) / 10 - 5);

    Controller1.Screen.setCursor(3, 17);
    Controller1.Screen.print("%1.0f ",
                             IntakeHalf.temperature(fahrenheit) / 10 - 5);
    Controller1.Screen.setCursor(3, 19);
    Controller1.Screen.print("%1.0f ", Arm.temperature(fahrenheit) / 10 - 5);
  }
}

int sensorsTask() {
  Optical.setLight(ledState::on);
  Optical.setLightPower(100, pct);
  while (1) {
    task::sleep(5);

    // GET gyro1 VALUE
    gyro1 = Inertial8.rotation(deg);

    // Get optical value
    ringDetected = Optical.isNearObject();
    redDetected = ((Optical.hue() > 330) || (Optical.hue() < 30));
    blueDetected = ((Optical.hue() < 250) && (Optical.hue() > 90));

    msecClock = Brain.timer(msec);
  }
}

int controlTask() {
  while (1) {
    sleep(10);
    axis1 = Controller1.Axis1.value();
    if (abs(axis1) < 15) {
      axis1 = 0;
    }
    axis2 = Controller1.Axis2.value();
    if (abs(axis2) < 15) {
      axis2 = 0;
    }
    axis3 = Controller1.Axis3.value();
    if (abs(axis3) < 15) {
      axis3 = 0;
    }
    axis4 = Controller1.Axis4.value();
    if (abs(axis4) < 15) {
      axis4 = 0;
    }

    leftSpeed = axis3 + axis1;
    rightSpeed = axis3 - axis1;
  }
}

int driveTask() {
  while (1) {
    if (driveHold) {
      LTopDrive.setStopping(hold);
      LBackDrive.setStopping(hold);
      RBackDrive.setStopping(hold);
      RTopDrive.setStopping(hold);
      LBottomDrive.setStopping(hold);
      RBottomDrive.setStopping(hold);
    } else {
      LTopDrive.setStopping(coast);
      LBackDrive.setStopping(coast);
      RBackDrive.setStopping(coast);
      RTopDrive.setStopping(coast);
      LBottomDrive.setStopping(coast);
      RBottomDrive.setStopping(coast);
    }

    LTopDrive.setMaxTorque(driveTorque, pct);
    LBackDrive.setMaxTorque(driveTorque, pct);
    RTopDrive.setMaxTorque(driveTorque, pct);
    RBackDrive.setMaxTorque(driveTorque, pct);
    LBottomDrive.setMaxTorque(driveTorque, pct);
    RBottomDrive.setMaxTorque(driveTorque, pct);

    LTopDrive.spin(fwd, leftSpeed, pct);
    LBackDrive.spin(fwd, leftSpeed, pct);
    LBottomDrive.spin(fwd, leftSpeed, pct);
    RTopDrive.spin(fwd, rightSpeed, pct);
    RBackDrive.spin(fwd, rightSpeed, pct);
    RBottomDrive.spin(fwd, rightSpeed, pct);

    sleep(6);
  }
}

int intakeControlTask() {
  while (1) {
    sleep(5);
    IntakeFull.setVelocity(func.conveyorSpeed, pct);
    IntakeHalf.setVelocity(func.rollerSpeed, pct);
    if (func.conveyorSpeed > 0 || func.rollerSpeed > 0) {
      intakeRunning = true;
    } else {
      intakeRunning = false;
    }
  }
}

int sortingTask() {
  // bool pausedForRed = false;
  // bool pausedForBlue = false;

  while (1) {
    sleep(5);
    // while(auto_started) {
    //   if (sortingColor == "red" && redDetected && intakeRunning &&
    //       !pausedForRed) {
    //     pausedForRed = true;
    //     if (func.conveyorSpeed != 0) {
    //       originalIntakeSpeed = func.conveyorSpeed;
    //     } else {
    //       func.conveyorSpeed = 100;
    //     }
    //     sleep(145);
    //     func.conveyorSpeed = 0;
    //     sleep(200);
    //     func.conveyorSpeed = originalIntakeSpeed;
    //   } else if (sortingColor == "blue" && blueDetected && intakeRunning &&
    //              !pausedForBlue) {
    //     pausedForBlue = true;
    //     if (func.conveyorSpeed != 0) {
    //       originalIntakeSpeed = func.conveyorSpeed;
    //     } else {
    //       func.conveyorSpeed = 100;
    //     }
    //     sleep(145);
    //     func.conveyorSpeed = 0;
    //     sleep(200);
    //     func.conveyorSpeed = originalIntakeSpeed;
    //   }

    //   if (!redDetected) {
    //     pausedForRed = false;
    //   }
    //   if (!blueDetected) {
    //     pausedForBlue = false;
    //   }
    // }
  }
}

int armStatesTask() {
  Arm.setStopping(hold);
  bool outakeHappened = false;

  while (1) {
    sleep(20);
    if (!ispreauto) {
      Arm.setVelocity((armGoal - Rotation2.position(deg)) * 1.75, pct);

      if (func.armState == 0) {
        armGoal = 1;
        outakeHappened = false;
      }
      if (func.armState == 1) {
        armGoal = 24;
        outakeHappened = false;
      }
      if (func.armState == 2 && !outakeHappened) {
        func.conveyorSpeed = -100;
        task::sleep(100);
        func.conveyorSpeed = 0;
        armGoal = 138;
        outakeHappened = true;
      }
    }
  }
}

void buttonLup_pressed() {
  if (Controller1.ButtonL2.pressing()) {
    func.rollerSpeed = -100;
    func.conveyorSpeed = 0;
  } else {
    if (func.armState < 2) {
      func.armState += 1;
    } else {
      func.armState = 2;
    }
  }
}

void buttonLdown_pressed() {
  if (Controller1.ButtonL1.pressing()) {
    func.rollerSpeed = -100;
    func.conveyorSpeed = 0;
  } else {
    if (func.armState > 0) {
      func.armState -= 1;
    } else {
      func.armState = 0;
    }
  }
}

void buttonLup_released() {func.rollerSpeed = 0;}

void buttonLdown_released() {func.rollerSpeed = 0;}

void buttonRup_pressed() {
  if (Controller1.ButtonR2.pressing()) {
    func.rollerSpeed = 100;
    func.conveyorSpeed = 0;
  } else {
    func.conveyorSpeed = 100;
    func.rollerSpeed = 100;
  }
}

void buttonRdown_pressed() {
  if (Controller1.ButtonR1.pressing()) {
    func.rollerSpeed = 100;
    func.conveyorSpeed = 0;
  } else {
    func.conveyorSpeed = -100;
    func.rollerSpeed = -100;
  }
}

void buttonRup_released() {
  func.conveyorSpeed = 0;
  func.rollerSpeed = 0;
}

void buttonRdown_released() {
  func.conveyorSpeed = 0;
  func.rollerSpeed = 0;
}

void buttonUP_pressed() {}

void buttonDOWN_pressed() {
  Arm.setVelocity(-70, pct);
  while (Arm.velocity(pct) > 1) {
    sleep(1);
  }
  Arm.setPosition(0, deg);
}

void buttonLEFT_pressed() { Doinker = !Doinker; }

void buttonRIGHT_pressed() {}

void buttonX_pressed() { IntakeLift = !IntakeLift; }

void buttonY_pressed() { MogoMech = !MogoMech; }

void buttonB_pressed() {
  func.conveyorSpeed = 100;
  func.rollerSpeed = 100;
  task::sleep(200);
  func.conveyorSpeed = 0;
  func.rollerSpeed = 0;
}

void buttonLup_pressed2() {}

void buttonLdown_pressed2() {}

void buttonRup_pressed2() {}

void buttonRdown_pressed2() {}

void buttonRdown_released2() {}

void buttonRup_released2() {}

/**
 * Auton function, which runs the selected auton. Case 0 is the default,
 * and will run in the brain screen goes untouched during preauton. Replace
 * drive_test(), for example, with your own auton function you created in
 * autons.cpp and declared in autons.h.
 */

void autonomous(void) {
  auto_started = true;
  switch (current_auton_selection) {
  case 0:
    sortingColor = "blue";
    redLeftNoWP();
    break;
  case 1:
    sortingColor = "red";
    blueRightNoWP();
    break;
  case 2:
    sortingColor = "blue";
    redGoalRush();
    break;
  case 3:
    sortingColor = "red";
    blueGoalRush();
    break;
  case 4:
    sortingColor = "blue";
    redLeftWP2();
    break;
  case 5:
    sortingColor = "red";
    blueRightWP2();
    break;
  case 6:
    sortingColor = "blue";
    redRightWP();
    break;
  case 7:
    sortingColor = "red";
    blueLeftWP();
    break;
  case 8:
    sortingColor = "blue";
    skillsAuton();
    break;
  }
}

void buttonA_pressed() { autonomous(); }

void usercontrol(void) {
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    // Replace this line with chassis.control_tank(); for tank drive
    // or chassis.control_holonomic(); for holo drive.
    chassis.control_arcade();

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
    auto_started = false;
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  task taskControllerScreen(controllerScreenTask);
  // task taskDrive(driveTask);
  // task taskControl(controlTask);
  task taskIntakeControl(intakeControlTask);
  task taskSensors(sensorsTask);
  task taskSorting(sortingTask);
  task taskArmStates(armStatesTask);

  Controller1.ButtonL1.pressed(buttonLup_pressed);
  Controller1.ButtonL2.pressed(buttonLdown_pressed);
  Controller1.ButtonL1.released(buttonLup_released);
  Controller1.ButtonL2.released(buttonLdown_released);
  Controller1.ButtonR1.pressed(buttonRup_pressed);
  Controller1.ButtonR2.pressed(buttonRdown_pressed);
  Controller1.ButtonR1.released(buttonRup_released);
  Controller1.ButtonR2.released(buttonRdown_released);
  Controller1.ButtonUp.pressed(buttonUP_pressed);
  Controller1.ButtonDown.pressed(buttonDOWN_pressed);
  Controller1.ButtonRight.pressed(buttonRIGHT_pressed);
  Controller1.ButtonLeft.pressed(buttonLEFT_pressed);
  Controller1.ButtonA.pressed(buttonA_pressed);
  Controller1.ButtonB.pressed(buttonB_pressed);
  Controller1.ButtonX.pressed(buttonX_pressed);
  Controller1.ButtonY.pressed(buttonY_pressed);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
