// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller
// RFDrive              motor         1
// RHalfW               motor         2
// RBDrive              motor         3
// LFDrive              motor         5
// LHalfW               motor         6
// LBDrive              motor         7
// Inertial14           inertial      14
// OdomX                rotation      8
// OdomY                rotation      9
// MogoMech             digital_out   A
// Optical18            optical       18
// SortingMech          digital_out   B
// RightPTOMotor        motor         12
// LeftPTOMotor         motor         13
// PTO1                 digital_out   C
// PTO2                 digital_out   D
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller
// RFDrive              motor         1
// RHalfW               motor         2
// RBDrive              motor         3
// LFDrive              motor         5
// LHalfW               motor         6
// LBDrive              motor         7
// Inertial14           inertial      14
// OdomX                rotation      8
// OdomY                rotation      9
// MogoMech             digital_out   A
// Optical18            optical       18
// SortingMech          digital_out   B
// RightPTOMotor        motor         12
// LeftPTO              motor         13
// PTO1                 digital_out   C
// PTO2                 digital_out   D
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller
// RFDrive              motor         1
// RHalfW               motor         2
// RBDrive              motor         3
// LFDrive              motor         5
// LHalfW               motor         6
// LBDrive              motor         7
// Inertial14           inertial      14
// OdomX                rotation      8
// OdomY                rotation      9
// MogoMech             digital_out   A
// Optical18            optical       18
// SortingMech          digital_out   B
// RightPTO             motor         12
// LeftPTO              motor         13
// PTO1                 digital_out   C
// PTO2                 digital_out   D
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "vex.h"
#include <iostream>
#include <math.h>

using namespace vex;
vex::competition Competition;

int fieldControlState = 0;
int axis1;
int axis2;
int axis3;
int axis4;

double gyro1;
int msecClock;
int headingMultiplier = 1;

double avgDriveDistance;
double avgDriveSpeed;
int slowestDrive;
int fastestDrive;
int leftSpeed = 0;
int rightSpeed = 0;
int driveTorque = 100;
int leftDriveTorque;
int rightDriveTorque;
bool driveHold = false;
float leftSpeedIncrement;
float rightSpeedIncrement;

double wheelDiameter = 2.0;
double wheelCircumference = wheelDiameter * M_PI;
double ticksPerRevolution = 360.0; // for v5 rotation sensor as they are sped
double x = 0.0;
double y = 0.0;
const int numReadings = 10;
double readingsX[numReadings];
double readingsY[numReadings]; // for sam: this just means that this variable is
                               // an array with a size of numReadings, which
                               // is 10.0, in the format of a double(same with
                               // readingsX). it is used in the odom function to
                               // find the average encoder value over 10 steps
int readIndex = 0;
double totalX = 0;
double totalY = 0;
double averageX = 0;
double averageY = 0;

bool ignoreRelease = false;
bool sorting = true;

bool autonRunning = false;
int autonNumber = 1;
bool autonHappened = false;

void sleep(int sleepmsec) {
  if (autonRunning) {
    task::sleep(sleepmsec);
  }
}

void resetTimer() {
  Brain.resetTimer();
  task::sleep(5);
}

void resetGyro() {
  Inertial14.setRotation(0, deg);
  task::sleep(5);
}

void setGyro(int Heading) {
  Inertial14.setRotation(Heading, deg);
  task::sleep(5);
}

void resetDrive() {
  LFDrive.resetPosition();
  RFDrive.resetPosition();
  LHalfW.resetPosition();
  RHalfW.resetPosition();
  LBDrive.resetPosition();
  RBDrive.resetPosition();
  LeftPTOMotor.resetPosition();
  RightPTOMotor.resetPosition();
  task::sleep(5);
}

void stopDrive() {
  leftSpeed = 0;
  rightSpeed = 0;
  task::sleep(5);
}

void stopAll() {
  LFDrive.stop();
  RFDrive.stop();
  LHalfW.stop();
  RHalfW.stop();
  LBDrive.stop();
  RBDrive.stop();
  LeftPTOMotor.stop();
  RightPTOMotor.stop();
  task::sleep(5);
}

void preAuton() {
  fieldControlState = 0;
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(320, 200, "Pre Auton");
  resetDrive();
  task::sleep(100);
  Controller1.Screen.clearScreen();
  fieldControlState = 1;
  LeftPTOMotor.spin(fwd);
  RightPTOMotor.spin(fwd);
}

int brainScreenTask() {
  while (1) {
    task::sleep(100);
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(1, 20, "LFMotor: %5.2f   ", gyro1);
    Brain.Screen.printAt(188, 20, "RFMotor: %5.2f   ", gyro1);
    Brain.Screen.printAt(1, 40, "LTMotor: %5.2f   ", gyro1);
    Brain.Screen.printAt(188, 40, "RTMotor: %5.2f   ", gyro1);
    Brain.Screen.printAt(1, 60, "Avg Motor Dist: %4.2f   ", avgDriveDistance);
    Brain.Screen.printAt(1, 100, "gyro1: %5.2f    ", gyro1);
    Brain.Screen.printAt(1, 140, "msecClock: %d    ", msecClock);
    Brain.Screen.printAt(1, 170, "       ");
    Brain.Screen.printAt(370, 20, "Axis1: %d", axis1);
    Brain.Screen.printAt(370, 40, "Axis2: %d", axis2);
    Brain.Screen.printAt(370, 60, "Axis3: %d", axis3);
    Brain.Screen.printAt(370, 80, "Axis4: %d", axis4);
    Brain.Screen.printAt(320, 110, "Reflect: %3.2f  ");
    Brain.Screen.printAt(320, 150, "PotR: %3.2f   ");

    if (autonNumber == 1) {
      Brain.Screen.printAt(1, 210, "Auton: Qual. Right");
      Brain.Screen.setFillColor(purple);
    } else if (autonNumber == 2) {
      Brain.Screen.printAt(1, 210, "Auton: Qual. Left");
      Brain.Screen.setFillColor("#2bffb5");
    } else if (autonNumber == 3) {
      Brain.Screen.printAt(1, 210, "Auton: Red Elims");
      Brain.Screen.setFillColor(red);
    } else if (autonNumber == 4) {
      Brain.Screen.printAt(1, 210, "Auton: Blue Elims");
      Brain.Screen.setFillColor("#00ecff");
    } else if (autonNumber == 5) {
      Brain.Screen.printAt(1, 210, "Auton: Auton Skills");
      Brain.Screen.setFillColor(yellow);
    } else if (autonNumber == 6) {
      Brain.Screen.printAt(1, 210, "Auton: Driver Skills");
      Brain.Screen.setFillColor("#ff5ac6");
    }

    if (fieldControlState == 0) {
      Brain.Screen.printAt(320, 200, "Pre Auton");
    }
    if (fieldControlState == 1) {
      Brain.Screen.printAt(320, 200, "Pre Auton Done");
    }
    if (fieldControlState == 2) {
      Brain.Screen.printAt(320, 200, "Autonomous");
    }
    if (fieldControlState == 3) {
      Brain.Screen.printAt(320, 200, "Autonomous Done");
    }
    if (fieldControlState == 4) {
      Brain.Screen.printAt(320, 200, "Driver");
    }
  }
}

int controllerScreenTask() {
  Controller1.Screen.clearScreen();
  while (1) {
    task::sleep(50);

    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Gyro: %3.2f  ", Inertial14.rotation());
    Controller1.Screen.setCursor(1, 15);
    if (sorting == false) {
      Controller1.Screen.print("B in");
    } else {
      Controller1.Screen.print("R in");
    }
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("%3.0f", Optical18.hue());

    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("%1.0f ",
                             LFDrive.temperature(fahrenheit) / 10 - 5);

    Controller1.Screen.setCursor(3, 3);
    Controller1.Screen.print("%1.0f ",
                             RFDrive.temperature(fahrenheit) / 10 - 5);

    Controller1.Screen.setCursor(3, 5);
    Controller1.Screen.print("%1.0f ",
                             LBDrive.temperature(fahrenheit) / 10 - 5);

    Controller1.Screen.setCursor(3, 7);
    Controller1.Screen.print("%1.0f ",
                             RBDrive.temperature(fahrenheit) / 10 - 5);

    Controller1.Screen.setCursor(3, 9);
    Controller1.Screen.print("%1.0f ", LHalfW.temperature(fahrenheit) / 10 - 5);

    Controller1.Screen.setCursor(3, 11);
    Controller1.Screen.print("%1.0f ", RHalfW.temperature(fahrenheit) / 10 - 5);

    Controller1.Screen.setCursor(3, 13);
    Controller1.Screen.print("%1.0f ",
                             RightPTOMotor.temperature(fahrenheit) / 10 - 5);

    Controller1.Screen.setCursor(3, 15);
    Controller1.Screen.print("%1.0f ",
                             LeftPTOMotor.temperature(fahrenheit) / 10 - 5);
  }
}

int sortingTask() {
  while (1) {
    if (sorting == false) { // eject red
      if ((Optical18.hue() > 330) && (Optical18.hue() < 30)) {
        SortingMech = true;
        task::sleep(50);
        SortingMech = false;
      }
    } else { // eject blue
      if ((Optical18.hue() < 270) && (Optical18.hue() > 140)) {
        SortingMech = true;
        task::sleep(50);
        SortingMech = false;
      }
    }
  }
}

int sensorsTask() {
  int x = 100;
  while (1) {
    task::sleep(5);
    // GET MOTOR ENCODERS AND SCALE THEM TO DISTANCE IN INCHES(450 RPM)
    avgDriveDistance = (LFDrive.position(deg) + RBDrive.position(deg)) * 0.012;

    // GET AVERAGE MOTOR SPEED PERCENTAGE
    avgDriveSpeed = (LFDrive.velocity(pct) + RBDrive.velocity(pct)) * .5;

    // GET gyro1 VALUE
    gyro1 = Inertial14.rotation(deg);

    // GET SLOWEST DRIVE MOTOR SPEED
    x = fabs(RFDrive.velocity(pct));
    if (x > fabs(LFDrive.velocity(pct))) {
      x = fabs(LFDrive.velocity(pct));
    }
    if (x > fabs(LBDrive.velocity(pct))) {
      x = fabs(LBDrive.velocity(pct));
    }
    if (x > fabs(RBDrive.velocity(pct))) {
      x = fabs(RBDrive.velocity(pct));
    }
    if (x > fabs(LHalfW.velocity(pct))) {
      x = fabs(LHalfW.velocity(pct));
    }
    if (x > fabs(RHalfW.velocity(pct))) {
      x = fabs(RHalfW.velocity(pct));
    }
    if (!PTO1 && !PTO2) {
      if (x > fabs(LeftPTOMotor.velocity(pct))) {
        x = fabs(LeftPTOMotor.velocity(pct));
      }
      if (x > fabs(RightPTOMotor.velocity(pct))) {
        x = fabs(RightPTOMotor.velocity(pct));
      }
    }
    slowestDrive = abs(x);

    x = fabs(RFDrive.velocity(pct));
    if (x < fabs(LFDrive.velocity(pct))) {
      x = fabs(LFDrive.velocity(pct));
    }
    if (x < fabs(LBDrive.velocity(pct))) {
      x = fabs(LBDrive.velocity(pct));
    }
    if (x < fabs(RBDrive.velocity(pct))) {
      x = fabs(RBDrive.velocity(pct));
    }
    if (x < fabs(LHalfW.velocity(pct))) {
      x = fabs(LHalfW.velocity(pct));
    }
    if (x < fabs(RHalfW.velocity(pct))) {
      x = fabs(RHalfW.velocity(pct));
    }
    if (!PTO1 && !PTO2) {
      if (x < fabs(LeftPTOMotor.velocity(pct))) {
        x = fabs(LeftPTOMotor.velocity(pct));
      }
      if (x < fabs(RightPTOMotor.velocity(pct))) {
        x = fabs(RightPTOMotor.velocity(pct));
      }
    }
    fastestDrive = abs(x);

    msecClock = Brain.timer(msec);
  }
}

int driveTask() {
  while (1) {
    if (driveHold) {
      LFDrive.setStopping(hold);
      LBDrive.setStopping(hold);
      RBDrive.setStopping(hold);
      RFDrive.setStopping(hold);
      LHalfW.setStopping(hold);
      RHalfW.setStopping(hold);

    } else {
      LFDrive.setStopping(coast);
      LBDrive.setStopping(coast);
      RBDrive.setStopping(coast);
      RFDrive.setStopping(coast);
      LHalfW.setStopping(coast);
      RHalfW.setStopping(coast);
    }

    LeftPTOMotor.setStopping(coast);
    RightPTOMotor.setStopping(coast);

    leftDriveTorque = driveTorque * 1.15;
    rightDriveTorque = driveTorque;

    LFDrive.setMaxTorque(leftDriveTorque, pct);
    LHalfW.setMaxTorque(leftDriveTorque, pct);
    LBDrive.setMaxTorque(leftDriveTorque, pct);
    RFDrive.setMaxTorque(rightDriveTorque, pct);
    RHalfW.setMaxTorque(rightDriveTorque, pct);
    RBDrive.setMaxTorque(rightDriveTorque, pct);
    if (!PTO1 && !PTO2) {
      LeftPTOMotor.setMaxTorque(leftDriveTorque, pct);
      RightPTOMotor.setMaxTorque(rightDriveTorque, pct);
    }

    LFDrive.spin(fwd, leftSpeed, pct);
    LBDrive.spin(fwd, leftSpeed, pct);
    LHalfW.spin(fwd, leftSpeed, pct);
    RFDrive.spin(fwd, rightSpeed, pct);
    RBDrive.spin(fwd, rightSpeed, pct);
    RHalfW.spin(fwd, rightSpeed, pct);
    if (!PTO1 && !PTO2) {
      LeftPTOMotor.spin(fwd, leftSpeed, pct);
      RightPTOMotor.spin(fwd, rightSpeed, pct);
    }
    task::sleep(6);
  }
}

int odometryTask() {
  while (1) {
    static double oldX = 0;
    static double oldY = 0;

    totalX -= readingsX[readIndex];
    totalY -= readingsY[readIndex];

    double dX = OdomX.position(degrees) - oldX;
    double dY = OdomY.position(degrees) - oldY;

    totalX += dX;
    totalY += dY;

    readingsX[readIndex] = dX;
    readingsY[readIndex] = dY;

    readIndex = (readIndex + 1);

    // if array ends, restart it
    if (readIndex >= numReadings) {
      readIndex = 0;
    }

    averageX = totalX / numReadings;
    averageY = totalY / numReadings;

    if (fabs(averageX) > 1000 || fabs(averageY) > 1000) {
      oldX = OdomX.position(degrees);
      oldY = OdomY.position(degrees);
      continue;
    }

    oldX = OdomX.position(degrees);
    oldY = OdomY.position(degrees);

    double distX = (averageX / ticksPerRevolution) * wheelCircumference;
    double distY = (averageY / ticksPerRevolution) * wheelCircumference;

    x += distX;
    y += distY;

    // Output the current position to the terminal
    printf("X: %.2f, Y: %.2f\n", x, y);

    task::sleep(10);
  }
  return 0;
}

void driveDistance(int Speed, double Distance, double Heading) {
  double kP = 0.2;
  double kI = 0.00;
  double kD = 0.01;
  double integral = 0;
  double previousError = 0;

  resetDrive();
  task::sleep(10);
  double rightTurnDifference;
  while ((fabs(avgDriveDistance) < Distance) && autonRunning) {
    double error = Heading - gyro1;
    integral += error;
    double derivative = error - previousError;
    rightTurnDifference = kP * error + kI * integral + kD * derivative;
    leftSpeed = Speed + rightTurnDifference;
    rightSpeed = Speed - rightTurnDifference;
    previousError = error;
    task::sleep(10);
  }
  stopDrive();
}

void driveTillStop(int Speed, double Heading) {
  resetDrive();
  resetTimer();
  task::sleep(10);
  int rightTurnDifference;
  while ((slowestDrive > 3 || msecClock < 500) && autonRunning) {
    rightTurnDifference = (Heading - gyro1) * .65;
    leftSpeed = Speed + rightTurnDifference;
    rightSpeed = Speed - rightTurnDifference;
    task::sleep(10);
  }
  stopDrive();
}

void driveTurn(int Speed, int Heading, int Accuracy) {
  int integral = 0;
  int previousError = 0;
  double kP = .415;
  double kI = 0;
  double kD = 0.005;
  double lsp;
  double rsp;

  int newHeading = Heading + Accuracy - 1;
  while ((fabs(newHeading - gyro1) > Accuracy ||
          (fabs(LFDrive.velocity(pct)) > 2.5)) &&
         autonRunning) {
    double error = newHeading - gyro1;
    integral += error;
    double derivativeTurn = error - previousError;
    double output = (kP * error) + (kI * integral) + (kD * derivativeTurn);
    lsp = +output;
    if (fabs(lsp) < 2) {
      lsp = 2 * fabs(lsp) / lsp;
    }
    leftSpeed = lsp;
    rsp = -output;
    if (fabs(rsp) < 2) {
      rsp = 2 * fabs(rsp) / rsp;
    }
    rightSpeed = rsp;
    previousError = error;
    task::sleep(5);
  }

  stopDrive();
  // integral = 0;
  // previousError = 0;
  task::sleep(10);
}

double Kp_turn = 10.00, Ki_turn = 0.0, Kd_turn = 0.0;
double turn_integral = 0.0, turn_prev_error = 0.0;
double Kp_fwd = 10.00, Ki_fwd = 0.0, Kd_fwd = 0.0;
double fwd_integral = 0.0, fwd_prev_error = 0.0;

void driveTo(int target_x, int target_y, int speed) {
  while (1) {
    double dx = target_x - x;
    double dy = target_y - y;
    double distance = sqrt(dx * dx + dy * dy);

    if (distance < 0.1)
      break; // Check early to avoid unnecessary calculations if already
             // at target

    double target_angle = atan2(dy, dx);
    double turn_error = target_angle - gyro1;

    while (turn_error > M_PI)
      turn_error -= 2 * M_PI;
    while (turn_error < -M_PI)
      turn_error += 2 * M_PI;

    double fwd_error = distance;
    double turn_output = Kp_turn * turn_error + Ki_turn * turn_integral -
                         Kd_turn * (turn_error - turn_prev_error);
    double fwd_output = Kp_fwd * fwd_error + Ki_fwd * fwd_integral -
                        Kd_fwd * (fwd_error - fwd_prev_error);

    turn_integral += turn_error;
    turn_prev_error = turn_error;

    fwd_integral += fwd_error;
    fwd_prev_error = fwd_error;

    leftSpeed = speed * fwd_output + turn_output;
    rightSpeed = speed * fwd_output - turn_output;

    task::sleep(5);
  }
}

void buttonLup_pressed() {}

void buttonLdown_pressed() {}

void buttonLup_released() {}

void buttonLdown_released() {}

void buttonRup_pressed() {
  if (!PTO1 && !PTO2) {
    PTO1 = true;
    PTO2 = true;
  } else {
    LeftPTOMotor.setVelocity(100, pct);
    RightPTOMotor.setVelocity(100, pct);
  }
  if (LeftPTOMotor.velocity(pct) <= 5 || RightPTOMotor.velocity(pct) <= 5) {
    LeftPTOMotor.setVelocity(-100, pct);
    RightPTOMotor.setVelocity(-100, pct);
    task::sleep(250);
    LeftPTOMotor.setVelocity(0, pct);
    RightPTOMotor.setVelocity(0, pct);
  }
}

void buttonRdown_pressed() {
  if (!PTO1 && !PTO2) {
    PTO1 = true;
    PTO2 = true;
  } else {
    LeftPTOMotor.setVelocity(-100, pct);
    RightPTOMotor.setVelocity(-100, pct);
  }
  if (LeftPTOMotor.velocity(pct) >= -5 || RightPTOMotor.velocity(pct) >= -5) {
    LeftPTOMotor.setVelocity(100, pct);
    RightPTOMotor.setVelocity(100, pct);
    task::sleep(250);
    LeftPTOMotor.setVelocity(0, pct);
    RightPTOMotor.setVelocity(0, pct);
  }
}

void buttonRup_released() {
  LeftPTOMotor.setVelocity(0, pct);
  RightPTOMotor.setVelocity(0, pct);
}

void buttonRdown_released() {
  LeftPTOMotor.setVelocity(0, pct);
  RightPTOMotor.setVelocity(0, pct);
}

void buttonUP_pressed() {}

void buttonDOWN_pressed() {}

void buttonRIGHT_pressed() {}

void buttonLEFT_pressed() {
  if (autonNumber > 6) {
    autonNumber = 1;
  } else {
    autonNumber += 1;
  }
  if (autonRunning) {
    autonRunning = false;
  }
}

void brain_pressed() {}

void buttonX_pressed() {
  PTO1 = !PTO1;
  PTO2 = !PTO2;
}

void buttonB_pressed() {}

void buttonY_pressed() {}

void buttonLup_pressed2() {}

void buttonLdown_pressed2() {}

void buttonRup_pressed2() {}

void buttonRdown_pressed2() {}

void buttonRdown_released2() {}

void buttonRup_released2() {}

void quals() {}

void elims() {}

void skillsAuton() {}

void skillsDriver() {}

void PIDTest() {
  driveTurn(40, 45, 10);
  sleep(1000);
  driveTurn(40, 135, 10);
  sleep(1000);
  driveTurn(40, 270, 10);
  sleep(1000);
  driveTurn(40, 450, 10);
  sleep(5);
}

void autonomous() {
  autonHappened = true;
  autonRunning = true;
  driveHold = true;
  if (autonNumber == 1) {
    headingMultiplier = 1;
    quals();
  } else if (autonNumber == 2) {
    headingMultiplier = -1;
    quals();
  } else if (autonNumber == 3) {
    headingMultiplier = 1;
    elims();
  } else if (autonNumber == 4) {
    headingMultiplier = -1;
    elims();
  } else if (autonNumber == 5) {
    skillsAuton();
  } else if (autonNumber == 6) {
    skillsDriver();
  }
}

void buttonA_pressed() {
  if (autonRunning) {
    autonRunning = false;
  } else if (!autonHappened) {
    autonomous();
  }
  driveTorque = 100;
  driveHold = false;
}

void usercontrol() {
  resetTimer();
  autonRunning = false;
  driveHold = false;
  driveTorque = 100;
  fieldControlState = 4;
  stopAll();
  while (1) {
    task::sleep(10);
    if (autonRunning == false) {
      axis1 = Controller1.Axis1.value();
      if (abs(axis1) < 5) {
        axis1 = 0;
      }
      axis2 = Controller1.Axis2.value();
      if (abs(axis2) < 5) {
        axis2 = 0;
      }
      axis3 = Controller1.Axis3.value();
      if (abs(axis3) < 5) {
        axis3 = 0;
      }
      axis4 = Controller1.Axis4.value();
      if (abs(axis4) < 5) {
        axis4 = 0;
      }

      leftSpeed = axis3 + axis1;

      rightSpeed = axis3 - axis1;

      if (LFDrive.velocity(pct) >= -20) {
        resetDrive();
      }
    }
  }
}

int main() {
  preAuton();
  task taskBrainScreen(brainScreenTask);
  task taskCntrlrScreen(controllerScreenTask);
  task taskSensors(sensorsTask);
  task taskDrive(driveTask);
  task taskOdometry(odometryTask);
  task taskSorting(sortingTask);
  Brain.Screen.pressed(brain_pressed);
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
}