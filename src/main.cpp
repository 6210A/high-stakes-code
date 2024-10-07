#include "vex.h"
#include <cmath>
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

bool release2 = false;
float armState = 0;
float armGoal = 0;

bool redirectActive = false;
bool delayTaskActive = false;
double hookLocation = 0;
bool intakeRedirecting = false;
bool stopIntake = false;
double currentPTO = 0;
int conveyorSpeed = 0;
int rollerSpeed = 0;

bool sortingColor = true;
bool sortingTaskActive = false;
bool ringDetected;
bool redDetected;
bool blueDetected;

bool intakeTaskOn;

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
  Inertial15.setRotation(0, deg);
  task::sleep(5);
}

void setGyro(int Heading) {
  Inertial15.setRotation(Heading, deg);
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
  armState = 0;
  sleep(300);
  IntakeLift = false;
  ClawPivot = false;
  IntakeRotation.setPosition(0, degrees);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(320, 200, "Pre Auton");
  resetDrive();
  task::sleep(100);
  Controller1.Screen.clearScreen();
  fieldControlState = 1;

  // Zero Arm Motors
  RightArm.spin(fwd);
  LeftArm.spin(fwd);
  RightArm.setVelocity(50, pct);
  LeftArm.setVelocity(50, pct);
  task::sleep(250);
  while (RightArm.velocity(pct) > 1) {
    task::sleep(5);
  }
  RightArm.setVelocity(0, pct);
  LeftArm.setVelocity(0, pct);
  task::sleep(100);
  RightArm.setPosition(0, deg);
  LeftArm.setPosition(0, deg);

  LeftPTOMotor.setVelocity(50, pct);
  LeftPTOMotor.setStopping(hold);
  LeftPTOMotor.spinFor(reverse, 1.917, turns, true);
  LeftPTOMotor.setStopping(coast);
  task::sleep(200);
  LeftPTOMotor.spin(fwd);
  RightPTOMotor.spin(fwd);
  RightPTOMotor.setVelocity(0, pct);
  LeftPTOMotor.setVelocity(0, pct);
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
    Controller1.Screen.print("G: %3.2f  ", Inertial15.rotation());
    Controller1.Screen.setCursor(1, 10);
    if (sortingColor) {
      Controller1.Screen.print("B in");
    } else {
      Controller1.Screen.print("R in");
    }
    Controller1.Screen.setCursor(1, 17);
    Controller1.Screen.print("%d", autonRunning);

    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("%1.0f ",
                             RightArm.temperature(fahrenheit) / 10 - 5);
    Controller1.Screen.setCursor(2, 3);
    Controller1.Screen.print("%1.0f ",
                             LeftArm.temperature(fahrenheit) / 10 - 5);
    Controller1.Screen.setCursor(2, 6);
    Controller1.Screen.print("State %1.0f", armState);
    Controller1.Screen.setCursor(2, 18);
    Controller1.Screen.print("%d", ringDetected);
    Controller1.Screen.setCursor(2, 20);
    Controller1.Screen.print("%d", redirectActive);

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

    Controller1.Screen.setCursor(3, 15);
    Controller1.Screen.print("%1.0f ",
                             RightPTOMotor.temperature(fahrenheit) / 10 - 5);

    Controller1.Screen.setCursor(3, 17);
    Controller1.Screen.print("%1.0f ",
                             LeftPTOMotor.temperature(fahrenheit) / 10 - 5);
  }
}

int armStatesTask() {
  LeftArm.setStopping(hold);
  RightArm.setStopping(hold);
  while (1) {
    task::sleep(5);
    LeftArm.spin(fwd,
                 ((-armGoal * 6) -
                  (LeftArm.position(deg) + RightArm.position(deg)) * .3),
                 pct);
    RightArm.spin(fwd,
                  ((-armGoal * 6) -
                   (LeftArm.position(deg) + RightArm.position(deg)) * .3),
                  pct);
    if (armState == 0) {
      armGoal = 0;
    }
    if (armState == 1) {
      armGoal = 15;
    }
    if (armState == 2) {
      armGoal = 35;
    }
    if (armState == 3)
      armGoal = 46;
  }
}

int intakeRotationTask() {
  while (1) {
    task::sleep(5);
    hookLocation = (fmod((IntakeRotation.position(turns) * 12), 23) / 23);
  }
}

int intakeSpeedTask() {
  while (1) {
    task::sleep(5);
    if (!PTO) {
      if (redirectActive) {
        conveyorSpeed = 50;
        rollerSpeed = 100;
      } else if (intakeRedirecting) {
        conveyorSpeed = -50;
        rollerSpeed = 0;
      } else if (stopIntake) {
        conveyorSpeed = 0;
        rollerSpeed = 0;
      }
      LeftPTOMotor.setVelocity(conveyorSpeed * -1, pct);
      RightPTOMotor.setVelocity(rollerSpeed, pct);
    }
  }
}

int sortingDelayTask() {
  delayTaskActive = true;
  currentPTO = fabs(IntakeRotation.position(deg));
  task::sleep(100);
  while (hookLocation < .95) {
    task::sleep(1);
  }
  stopIntake = true;
  task::sleep(150);
  stopIntake = false;
  conveyorSpeed = 100;
  rollerSpeed = 100;
  delayTaskActive = false;
  return 0;
}

int redirectDelayTask() {
  delayTaskActive = true;
  currentPTO = fabs(IntakeRotation.position(deg));
  task::sleep(100);
  while (hookLocation < .68) {
    task::sleep(1);
  }
  intakeRedirecting = true;
  task::sleep(200);
  intakeRedirecting = false;
  delayTaskActive = false;
  return 0;
}

int intakeTask() {
  Optical.setLight(ledState::on);
  Optical.setLightPower(100, pct);
  while (1) {
    task::sleep(5);

    if (Controller1.ButtonR1.pressing() || autonRunning) {
      // Redirect
      if (redirectActive && ringDetected && !delayTaskActive) {
        task taskRedirectDelay(redirectDelayTask);
      }
    }

    // Sorting
    if (ringDetected) {
      if (sortingColor) { // eject red
        if (redDetected && !delayTaskActive) {
          task taskSortingDelay(sortingDelayTask);
        }
      } else { // eject blue
        if (blueDetected && !delayTaskActive) {
          task taskSortingDelay(sortingDelayTask);
        }
      }
    }
  }
}

int sensorsTask() {
  int x = 100;
  while (1) {
    task::sleep(5);
    // GET MOTOR ENCODERS AND SCALE THEM TO DISTANCE IN INCHES (450 RPM)
    avgDriveDistance =
        (LFDrive.position(deg) + RBDrive.position(deg)) * 0.00955;

    // GET AVERAGE MOTOR SPEED PERCENTAGE
    avgDriveSpeed = (LFDrive.velocity(pct) + RBDrive.velocity(pct)) * .5;

    // GET gyro1 VALUE
    gyro1 = Inertial15.rotation(deg);

    // Get optical value
    ringDetected = Optical.isNearObject();
    redDetected = ((Optical.hue() > 330) || (Optical.hue() < 30));
    blueDetected = ((Optical.hue() < 250) && (Optical.hue() > 90));

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
    if (PTO) {
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
    if (PTO) {
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
    if (PTO) {
      LeftPTOMotor.setMaxTorque(leftDriveTorque, pct);
      RightPTOMotor.setMaxTorque(rightDriveTorque, pct);
    }

    LFDrive.spin(fwd, leftSpeed, pct);
    LBDrive.spin(fwd, leftSpeed, pct);
    LHalfW.spin(fwd, leftSpeed, pct);
    RFDrive.spin(fwd, rightSpeed, pct);
    RBDrive.spin(fwd, rightSpeed, pct);
    RHalfW.spin(fwd, rightSpeed, pct);
    if (PTO) {
      LeftPTOMotor.spin(fwd, leftSpeed, pct);
      RightPTOMotor.spin(fwd, rightSpeed, pct);
    }
    task::sleep(6);
  }
}

void driveDistance(int Speed, double Distance, double Heading) {
  double kP = 0.6;
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

void driveDistanceAcceleration(int initialSpeed, int peakSpeed, int endingSpeed,
                               double Distance, double Heading) {
  double kP = 0.6;
  double kI = 0.00;
  double kD = 0.01;
  double integral = 0;
  double previousError = 0;

  resetDrive();
  task::sleep(10);
  double rightTurnDifference;
  double currentSpeed = initialSpeed;
  double accelerationDistance =
      Distance * 0.3; // 30% of the distance for acceleration
  double decelerationDistance =
      Distance * 0.3; // 30% of the distance for deceleration
  double constantSpeedDistance =
      Distance - accelerationDistance - decelerationDistance;

  while ((fabs(avgDriveDistance) < Distance) && autonRunning) {
    double error = Heading - gyro1;
    integral += error;
    double derivative = error - previousError;
    rightTurnDifference = kP * error + kI * integral + kD * derivative;

    // Acceleration phase
    if (fabs(avgDriveDistance) < accelerationDistance) {
      currentSpeed =
          initialSpeed + (peakSpeed - initialSpeed) *
                             (fabs(avgDriveDistance) / accelerationDistance);
    }
    // Constant speed phase
    else if (fabs(avgDriveDistance) <
             (accelerationDistance + constantSpeedDistance)) {
      currentSpeed = peakSpeed;
    }
    // Deceleration phase
    else {
      currentSpeed =
          peakSpeed - (peakSpeed - endingSpeed) *
                          ((fabs(avgDriveDistance) - accelerationDistance -
                            constantSpeedDistance) /
                           decelerationDistance);
    }

    leftSpeed = currentSpeed + rightTurnDifference;
    rightSpeed = currentSpeed - rightTurnDifference;
    previousError = error;
    task::sleep(10);
  }
  stopDrive();
}

void driveTurn(int Heading, int Accuracy) {
  int integral = 0;
  int previousError = 0;
  double kP;
  double kI;
  double kD;
  double lsp;
  double rsp;

  if (MogoMech) {
    kP = .504;
    kI = 0;
    kD = 0.005;
  } else {
    kP = .5;
    kI = 0;
    kD = 0.005;
  }

  int newHeading = Heading + Accuracy;
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
    sleep(5);
  }

  stopDrive();
  // integral = 0;
  // previousError = 0;
  sleep(10);
}

int driveArc(int Speed, int Distance, int Heading, int Radius) {
  int integral = 0;
  int previousError = 0;
  double kP = 0.415; // scales it to inches
  double kI = 0;
  double kD = 0.005;
  double lsp;
  double rsp;

  // define the target variables
  double targetDistance = Distance;
  double targetHeading = Heading;
  int currentDistance = 0;

  // safeguard to prevent invalid inputs
  if (Radius == 0 || Speed == 0 || Distance == 0) {
    return -1;
  }

  // calculate error
  while (currentDistance < targetDistance && autonRunning) {
    double error = targetHeading - gyro1;
    integral += error;
    double derivativeTurn = error - previousError;
    double output = (kP * error) + (kI * integral) + (kD * derivativeTurn);

    // calculate the actual left and right speeds based on the inputted radius
    // and velocity and the calculated output
    lsp = Speed * (1 - output / Radius);
    rsp = Speed * (1 + output / Radius);

    // safeguard to prevent stalling
    if (fabs(lsp) < 2) {
      lsp = 2 * fabs(lsp) / lsp;
    }
    if (fabs(rsp) < 2) {
      rsp = 2 * fabs(rsp) / rsp;
    }

    // set the motor speeds
    leftSpeed = lsp;
    rightSpeed = rsp;

    // update error and distance
    previousError = error;
    currentDistance =
        (LFDrive.position(deg) + RBDrive.position(deg)) / 2 * 0.0128;

    task::sleep(5);
  }
  stopDrive();
  task::sleep(10);

  return 0;
}

void buttonLup_pressed() {
  if (Controller1.ButtonL2.pressing()) {
    Doinker = true;
  }
  if (armState < 4) {
    armState += 1;
  } else {
    armState = 4;
  }
}

void buttonLdown_pressed() {
  if (Controller1.ButtonL1.pressing()) {
    Doinker = true;
  }
  if (armState > 0) {
    armState -= 1;
  } else {
    armState = 0;
  }
}

void buttonLup_released() { Doinker = false; }

void buttonLdown_released() { Doinker = false; }

void buttonRup_pressed() {
  if (Controller1.ButtonR2.pressing()) {
    release2 = true;
    redirectActive = true;
  } else {
    PTO = false;
    conveyorSpeed = 100;
    rollerSpeed = 100;
  }
}

void buttonRdown_pressed() {
  if (Controller1.ButtonR1.pressing()) {
    release2 = true;
    redirectActive = true;
  } else {
    PTO = false;
    conveyorSpeed = -50;
    rollerSpeed = -50;
  }
}

void buttonRup_released() {
  if (release2) {
    release2 = false;
    redirectActive = false;
  }
  conveyorSpeed = 0;
  rollerSpeed = 0;
}

void buttonRdown_released() {
  if (release2) {
    release2 = false;
    redirectActive = false;
  }
  conveyorSpeed = 0;
  rollerSpeed = 0;
}

void buttonUP_pressed() { sortingColor = !sortingColor; }

void buttonDOWN_pressed() { ClawPivot = !ClawPivot; }

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

void buttonX_pressed() { PTO = !PTO; }

void buttonA_pressed() { ClawPivot = !ClawPivot; }

void buttonY_pressed() { MogoMech = !MogoMech; }

void buttonB_pressed() { IntakeLift = !IntakeLift; }

void buttonLup_pressed2() {}

void buttonLdown_pressed2() {}

void buttonRup_pressed2() {}

void buttonRdown_pressed2() {}

void buttonRdown_released2() {}

void buttonRup_released2() {}

void frontAuton4() {
  stopIntake = false;
  setGyro(-75 * headingMultiplier);
  armState = 2;
  IntakeLift = false;
  conveyorSpeed = 0;
  rollerSpeed = 75;
  sleep(100);
  driveTurn(-160 * headingMultiplier, 3);
  rollerSpeed = 100;
  ClawPivot = true;
  conveyorSpeed = 100;
  if (headingMultiplier == 1) {
    while (!redDetected) {
      sleep(1);
    }
  } else if (headingMultiplier == -1) {
    while (!blueDetected) {
      sleep(1);
    }
  }
  conveyorSpeed = 0;
  driveDistance(35, 10.5, -155 * headingMultiplier);
  armState = 1;
  sleep(150);
  rollerSpeed = 0;
  driveDistance(-80, 44, -160 * headingMultiplier);
  MogoMech = true;
  sleep(50);
  driveDistance(70, 2, -160);
  conveyorSpeed = 100;
  rollerSpeed = 100;
  sleep(100);
  driveTurn(90 * headingMultiplier, 2);
  driveDistance(50, 18, 90 * headingMultiplier);
  rollerSpeed = 100;
  sleep(5);
  conveyorSpeed = 100;
  sleep(5);
  MogoMech = true;
  ClawPivot = true;
  // go to middle
  sleep(5);
  driveTurn(-75 * headingMultiplier, 3);
  driveDistance(40, 17, -70 * headingMultiplier);
  conveyorSpeed = 0;
  rollerSpeed = 0;
  driveDistance(40, 18, -65 * headingMultiplier);
  Doinker = true;
  sleep(100);
  driveDistance(-30, 12, -90 * headingMultiplier);
  Doinker = false;
  driveTurn(-67, 3);
  rollerSpeed = 100;
  conveyorSpeed = 100;
  driveDistance(40, 6, -67);
  sleep(200);
  driveDistance(-30, 6, -67);
}

void frontAuton5() {
    stopIntake = false;
  setGyro(-75 * headingMultiplier);
  armState = 2;
  IntakeLift = false;
  conveyorSpeed = 0;
  rollerSpeed = 75;
  sleep(100);
  driveTurn(-160 * headingMultiplier, 3);
  rollerSpeed = 100;
  ClawPivot = true;
  conveyorSpeed = 100;
  if (headingMultiplier == 1) {
    while (!redDetected) {
      sleep(1);
    }
  } else if (headingMultiplier == -1) {
    while (!blueDetected) {
      sleep(1);
    }
  }
  conveyorSpeed = 0;
  driveDistance(35, 11.5, -155 * headingMultiplier);
  armState = 1;
  sleep(150);
  rollerSpeed = 0;
  driveDistance(-80, 44, -160 * headingMultiplier);
  MogoMech = true;
  sleep(50);
  driveDistance(70, 2, -160);
  conveyorSpeed = 100;
  rollerSpeed = 100;
  sleep(100);
  driveTurn(90 * headingMultiplier, 2);
  driveDistance(50, 18, 90 * headingMultiplier);
  rollerSpeed = 100;
  sleep(5);
  conveyorSpeed = 100;
  sleep(5);
  MogoMech = true;
  ClawPivot = true;
  // go to middle
  sleep(5);
  driveTurn(-75 * headingMultiplier, 3);
  driveDistance(40, 17, -70 * headingMultiplier);
  conveyorSpeed = 0;
  rollerSpeed = 0;
  driveDistance(40, 18, -65 * headingMultiplier);
  Doinker = true;
  sleep(100);
  driveDistance(-30, 12, -90 * headingMultiplier);
  Doinker = false;
  driveTurn(-67, 3);
  rollerSpeed = 100;
  conveyorSpeed = 100;
  driveDistance(40, 6, -67);
  sleep(200);
  driveDistance(-30, 6, -67);
  sleep(400);
  armState = 3;
  rollerSpeed = 0;
  conveyorSpeed = 0;
  driveTorque = 75;
  sleep(300);
  driveTillStop(70, -67);
  armState = 1;
}

// void frontAuton4() {
//   setGyro(-148 * headingMultiplier);
//   stopIntake = true;
//   ClawPivot = true;
//   armState = 2;
//   sleep(300);
//   driveDistance(30, 7, -150 * headingMultiplier);
//   sleep(300);
//   armState = 1;
//   sleep(50);
//   driveDistance(-65, 22, -148 * headingMultiplier);
//   driveTurn(-180, 3);
//   driveDistance(-30, 13, -180);
//   MogoMech = true;
//   IntakeLift = false;
//   armState = 3;
//   driveTurn(-270 * headingMultiplier, 2);
//   setGyro(90);
//   stopIntake = false;
//   conveyorSpeed = 100;
//   rollerSpeed = 100;
//   sleep(100);
//   driveDistance(30, 24, 90 * headingMultiplier);
//   driveDistance(-80, 10, 90);
//   driveTurn(180 * headingMultiplier, 3);
//   driveTorque = 75;
//   driveTillStop(40, 180 * headingMultiplier);
//   driveDistance(-20, 3, 180);
//   driveTurn(100, 7);
//   Doinker = true;
//   driveDistance(40, 25, 95);
//   driveTurn(15, 7);
// }

void backAuton4() {
  setGyro(154);
  ClawPivot = true;
  armState = 2;
  IntakeLift = false;
  sleep(300);
  driveDistance(50, 5, 154);
  driveTorque = 75;
  driveTillStop(35, 110);
  armState = 1;
  sleep(300);
  driveTorque = 100;
  driveDistance(-50, 34, 154);
  armState = 0;
  MogoMech = true;
  driveTurn(270, 5);
  conveyorSpeed = 100;
  rollerSpeed = 100;
  driveDistance(50, 14, 270);
  sleep(100);
  driveDistance(70, 5, 270);
  sleep(300);
  stopIntake = true;
  driveDistance(-50, 12, 270);
  driveTurn(330, 5);
  stopIntake = false;
  conveyorSpeed = 100;
  rollerSpeed = 100;
  armState = 1;
  driveDistance(50, 15, 330);
  sleep(350);
  driveDistance(-50, 12, 330);
  driveTurn(270, 5);
  driveDistance(30, 12, 270);
  driveTurn(360, 5);
  driveDistance(30, 12, 360);
  sleep(350);
  driveDistance(-70, 10, 360);
  driveTurn(448, 5);
  driveDistance(45, 40, 448);
}

void skillsAuton() {}

void skillsDriver() {}

void PIDTest() {
  setGyro(0);
  driveTurn(180, 3);
  driveDistance(50, 1, 180);
}

void autonomous() {
  autonHappened = true;
  autonRunning = true;
  driveHold = true;
  if (autonNumber == 1) {
    IntakeLift = true;
    sortingColor = false;
    headingMultiplier = 1;
    frontAuton5();
    // backAuton4();
  } else if (autonNumber == 2) {
    IntakeLift = true;
    sortingColor = true;
    headingMultiplier = -1;
    frontAuton5();
  } else if (autonNumber == 3) {
    IntakeLift = true;
    sortingColor = false;
    headingMultiplier = 1;
    frontAuton4();
  } else if (autonNumber == 4) {

  } else if (autonNumber == 5) {

  } else if (autonNumber == 6) {
  }
}

void buttonRIGHT_pressed() {
  if (autonRunning) {
    autonRunning = false;
    driveHold = false;
  } else if (!autonHappened) {
    autonomous();
  }
  driveTorque = 100;
}

void usercontrol() {
  resetTimer();
  autonRunning = false;
  driveHold = false;
  IntakeLift = false;
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
  task taskArmStates(armStatesTask);
  task taskIntake(intakeTask);
  task taskIntakeRotation(intakeRotationTask);
  task taskIntakeSpeed(intakeSpeedTask);
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