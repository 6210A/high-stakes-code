// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller
// RFDrive              motor         1
// RMDrive              motor         2
// RBDrive              motor         3
// LFDrive              motor         5
// LMDrive              motor         6
// LBDrive              motor         7
// Inertial14           inertial      14
// Claw                 motor         15
// OdomX                rotation      8
// OdomY                rotation      9
// ClawFlip             digital_out   E
// BottomClaw           digital_out   B
// TopClaw              digital_out   C
// MogoMech             digital_out   D
// Arm1                 motor         11
// Arm2                 motor         12
// HangLock             digital_out   F
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller
// RFDrive              motor         1
// RMDrive              motor         2
// RBDrive              motor         3
// LFDrive              motor         5
// LMDrive              motor         6
// LBDrive              motor         7
// Inertial14           inertial      14
// Claw                 motor         10
// OdomX                rotation      8
// OdomY                rotation      9
// ClawFlip             digital_out   E
// BottomClaw           digital_out   B
// TopClaw              digital_out   C
// MogoMech             digital_out   D
// Arm1                 motor         11
// Arm2                 motor         12
// HangLock             digital_out   F
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller
// RFDrive              motor         1
// RMDrive              motor         2
// RBDrive              motor         3
// LFDrive              motor         5
// LMDrive              motor         6
// LBDrive              motor         7
// Inertial14           inertial      14
// Claw                 motor         10
// OdomX                rotation      8
// OdomY                rotation      9
// ClawFlip             digital_out   E
// BottomClaw           digital_out   B
// TopClaw              digital_out   C
// MogoMech             digital_out   D
// Arm1                 motor         11
// Arm2                 motor         12
// HangLock             digital_out   F
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller
// RFDrive              motor         1
// RMDrive              motor         2
// RBDrive              motor         3
// LFDrive              motor         5
// LMDrive              motor         6
// LBDrive              motor         7
// Inertial14           inertial      14
// Claw                 motor         10
// OdomX                rotation      8
// OdomY                rotation      9
// ClawFlip             digital_out   E
// BottomClaw                digital_out   B
// TopClaw                digital_out   C
// MogoMech             digital_out   D
// Arm1                 motor         11
// Arm2                 motor         12
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
double tilt;
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
bool clawStateThree;
bool clawStateFour;
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

int armSpeed = 0;
int clawSpeed = 0;
int clawState = 0;
bool clawStatesActive = true;
double armGoal = 0;
double clawGoal = 0;
bool ignoreRelease = false;

bool autonRunning = false;
int autonNumber = 3;
bool autonHappened = false;

void sleep(int sleepmsec) { task::sleep(sleepmsec); }

void resetTimer() {
  Brain.resetTimer();
  sleep(5);
}

void resetGyro() {
  Inertial14.setRotation(0, deg);
  sleep(5);
}

void setGyro(int Heading) {
  Inertial14.setRotation(Heading, deg);
  sleep(5);
}

void resetDrive() {
  LFDrive.resetPosition();
  RFDrive.resetPosition();
  LMDrive.resetPosition();
  RMDrive.resetPosition();
  LBDrive.resetPosition();
  RBDrive.resetPosition();
  sleep(5);
}

void stopDrive() {
  leftSpeed = 0;
  rightSpeed = 0;
  sleep(5);
}

void stopAll() {
  LFDrive.stop();
  RFDrive.stop();
  LMDrive.stop();
  RMDrive.stop();
  LBDrive.stop();
  RBDrive.stop();
  sleep(5);
}

void preAuton() {
  BottomClaw = true;
  HangLock = true;
  fieldControlState = 0;
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(320, 200, "Pre Auton");
  resetDrive();
  sleep(100);
  Controller1.Screen.clearScreen();
  fieldControlState = 1;
  Arm1.setVelocity(65, pct);
  Arm2.setVelocity(65, pct);
  Claw.setVelocity(75, pct);
  Arm1.spin(fwd);
  Arm2.spin(fwd);
  Claw.spin(fwd);
  sleep(200);
  while (Arm1.velocity(pct) > 0.1 || Arm2.velocity(pct) > 0.1 ||
         Claw.velocity(pct) > 0.1) {
    sleep(25);
  }
  sleep(50);
  Arm1.setPosition(700, deg);
  Arm2.setPosition(700, deg);
  Claw.setPosition(501, deg);
}

int brainScreenTask() {
  while (1) {
    sleep(100);
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
      Brain.Screen.printAt(1, 210, "Auton: Skills");
      Brain.Screen.setFillColor(yellow);
    } else if (autonNumber == 6) {
      Brain.Screen.printAt(1, 210, "Auton: Test");
      Brain.Screen.setFillColor(black);
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
    sleep(50);

    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Gyro: %3.2f  ", tilt); // Inertial14.rotation());
    Controller1.Screen.setCursor(1, 15);
    Controller1.Screen.print("A: %3.0f   ", armGoal);

    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("State: %d    ", clawState);
    Controller1.Screen.setCursor(2, 15);
    Controller1.Screen.print("C: %3.0f   ", clawGoal);

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
    Controller1.Screen.print("%1.0f ",
                             LMDrive.temperature(fahrenheit) / 10 - 5);
    Controller1.Screen.setCursor(3, 11);
    Controller1.Screen.print("%1.0f ",
                             RMDrive.temperature(fahrenheit) / 10 - 5);
    Controller1.Screen.setCursor(3, 13);
    Controller1.Screen.print("%1.0f ", Arm1.temperature(fahrenheit) / 10 - 5);
    Controller1.Screen.setCursor(3, 15);
    Controller1.Screen.print("%1.0f ", Arm2.temperature(fahrenheit) / 10 - 5);
    Controller1.Screen.setCursor(3, 17);
    Controller1.Screen.print("%1.0f ", Claw.temperature(fahrenheit) / 10 - 5);
  }
}

int sensorsTask() {
  int x = 100;
  while (1) {
    sleep(5);
    // GET MOTOR ENCODERS AND SCALE THEM TO DISTANCE IN INCHES(450 RPM)
    avgDriveDistance = (LFDrive.position(deg) + RMDrive.position(deg)) * 0.012;

    // GET AVERAGE MOTOR SPEED PERCENTAGE
    avgDriveSpeed = (LFDrive.velocity(pct) + RMDrive.velocity(pct)) * .5;

    // GET gyro1 VALUE
    gyro1 = Inertial14.rotation(deg);
    tilt = Inertial14.orientation(roll, deg) - 3;

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
    if (x > fabs(LMDrive.velocity(pct))) {
      x = fabs(LMDrive.velocity(pct));
    }
    if (x > fabs(RMDrive.velocity(pct))) {
      x = fabs(RMDrive.velocity(pct));
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
    if (x < fabs(LMDrive.velocity(pct))) {
      x = fabs(LMDrive.velocity(pct));
    }
    if (x < fabs(RMDrive.velocity(pct))) {
      x = fabs(RMDrive.velocity(pct));
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
      LMDrive.setStopping(hold);
      RMDrive.setStopping(hold);
    } else {
      LFDrive.setStopping(coast);
      LBDrive.setStopping(coast);
      RBDrive.setStopping(coast);
      RFDrive.setStopping(coast);
      LMDrive.setStopping(coast);
      RMDrive.setStopping(coast);
    }

    leftDriveTorque = driveTorque * 1.15;
    rightDriveTorque = driveTorque;

    LFDrive.setMaxTorque(leftDriveTorque, pct);
    LMDrive.setMaxTorque(leftDriveTorque, pct);
    LBDrive.setMaxTorque(leftDriveTorque, pct);
    RFDrive.setMaxTorque(rightDriveTorque, pct);
    RMDrive.setMaxTorque(rightDriveTorque, pct);
    RBDrive.setMaxTorque(rightDriveTorque, pct);

    LFDrive.spin(fwd, leftSpeed, pct);
    LBDrive.spin(fwd, leftSpeed, pct);
    LMDrive.spin(fwd, leftSpeed, pct);
    RFDrive.spin(fwd, rightSpeed, pct);
    RBDrive.spin(fwd, rightSpeed, pct);
    RMDrive.spin(fwd, rightSpeed, pct);

    sleep(6);
  }
}

int clawSpeedTask() {
  Arm1.setVelocity(0, pct);
  Arm2.setVelocity(0, pct);
  Claw.setVelocity(0, pct);
  Arm1.spin(forward);
  Arm2.spin(forward);
  Claw.spin(forward);
  while (1) {
    sleep(5);
  }
}

int clawStatesTask() {
  while (1) {
    sleep(5);

    if (armGoal > 97) {
      armGoal = 97;
    }

    if (armGoal < 3) {
      armGoal = 3;
    }
    if (clawGoal < -160) {
      clawGoal = -160;
    }
    // if (!autonRunning) {
    //   if (clawGoal > (armGoal * .8)) {
    //     clawGoal = (armGoal * .8);
    //   }
    //}

    Arm1.spin(fwd, (((armGoal * 7) - Arm1.position(deg)) * .35), pct);
    Arm2.spin(fwd, (((armGoal * 7) - Arm2.position(deg)) * .35), pct);
    Claw.spin(fwd, (((clawGoal * 5) - Claw.position(deg)) * .5), pct);
    if (clawStatesActive) {
      if (clawState == 0) {
        armGoal = 10;
        clawGoal = -90;
      }
      if (clawState == 1) {
        if (!MogoMech) {
          armGoal = 1;
          clawGoal = -5;
        } else {
          armGoal = 10;
          clawGoal = 10;
        }
      }
      if (clawState == 2) {
        armGoal = 45;
        clawGoal = 30;
      }
      if (clawState == 3) {
        armGoal = 90;
        clawGoal = 65;
      }
      if (clawState == 4) {
        armGoal = 97;
        clawGoal = 35;
      }
      if (clawState == 11) { // Grabs 2nd and 4th ring in stack of 4
        armGoal = 12;
        clawGoal = 0;
      }
      if (clawState == 12) { // Dump out 3rd ring
        armGoal = 45;
        clawGoal = 150;
      }
      if (clawState == 13) { // Claw flip
        armGoal = 30;
        clawGoal = 18;
      }
      if (clawState == 14) { // State 2 but closer to scoring
        armGoal = 43;
        clawGoal = -70;
      }
      if (clawState == 15) { // Flat against the ground
        armGoal = 10;
        clawGoal = 15;
      }
      if (clawState == 21) { // Score internally
        TopClaw = true;
        BottomClaw = true;
        armGoal = 46;
        clawGoal = -155;
      }
      if (clawState == 22) { // Above mobile goal (for goal rush)
        armGoal = 43;
        clawGoal = -28;
      }
      if (clawState == 23) { // Score on mobile goal externally from state 22
        armGoal = 40;
        clawGoal = 25;
      }
      if (clawState == 101) { // Use after state 21
        armGoal = 30;
        TopClaw = false;
        BottomClaw = false;
      }
      if (clawState == 102) {
        armGoal = 30;
      }
    } else {
      if (Controller1.ButtonUp.pressing()) {
        armGoal += .4;
      } else if (Controller1.ButtonDown.pressing()) {
        armGoal -= .4;
      }
      if (Controller1.ButtonX.pressing()) {
        clawGoal -= .5;
      } else if (Controller1.ButtonB.pressing()) {
        clawGoal += .5;
      }
    }
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

    sleep(10);
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
  sleep(10);
  double rightTurnDifference;
  while ((fabs(avgDriveDistance) < Distance) && autonRunning) {
    double error = Heading - gyro1;
    integral += error;
    double derivative = error - previousError;
    rightTurnDifference = kP * error + kI * integral + kD * derivative;
    leftSpeed = Speed + rightTurnDifference;
    rightSpeed = Speed - rightTurnDifference;
    previousError = error;
    sleep(10);
  }
  stopDrive();
}

void driveTillStop(int Speed, double Heading) {
  resetDrive();
  resetTimer();
  sleep(10);
  int rightTurnDifference;
  while ((slowestDrive > 3 || msecClock < 500) && autonRunning) {
    rightTurnDifference = (Heading - gyro1) * .65;
    leftSpeed = Speed + rightTurnDifference;
    rightSpeed = Speed - rightTurnDifference;
    sleep(10);
  }
  stopDrive();
}

void driveTurn(int Speed, int Heading, int Accuracy) {
  int integral = 0;
  int previousError = 0;
  double kP = .415;
  if (clawState == 3 || clawState == 4) {
    kP = 0.5;
  }
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
    sleep(5);
  }

  stopDrive();
  // integral = 0;
  // previousError = 0;
  sleep(10);
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

    sleep(5);
  }
}

void leftDoublePress() {
  ignoreRelease = true;
  if (clawState == 1) {
    clawState = 11;
  } else if (clawState == 21) {
    TopClaw = false;
    clawState = 102;
    sleep(250);
    clawState = 1;
  } else if (clawState == 2 || clawState == 11) {
    clawState = 21;
  }
}

void buttonLup_pressed() {
  clawStatesActive = true;
  if (Controller1.ButtonL2.pressing()) {
      leftDoublePress();
  } else {
    ignoreRelease = false;
  }
}

void buttonLdown_pressed() {
  clawStatesActive = true;
  if (Controller1.ButtonL1.pressing()) {
      leftDoublePress();
  } else {
    ignoreRelease = false;
  }
}

void buttonLup_released() {
  if (!ignoreRelease) {
    if (clawState == 21) {
      clawState = 101;
      sleep(250);
      clawState = 1;
    } else if (clawState == 11 || clawState == 12) {
      clawState = 1;
    } else {
      if (clawState < 4) {
        clawState += 1;
      } else {
        clawState = 4;
      }
    }
  }
}

void buttonLdown_released() {
  if (!ignoreRelease) {
    if (clawState == 21) {
      clawState = 101;
      sleep(250);
      clawState = 1;
    } else if (clawState == 11 || clawState == 12) {
      clawState = 1;
    } else {
      if (clawState > 1) {
        clawState -= 1;
      } else {
        clawState = 1;
      }
    }
  }
}

void buttonRup_pressed() {
  if (Controller1.ButtonR2.pressing()) {
    ignoreRelease = true;
    if (clawState == 1) {
      clawState = 13;
      ClawFlip = !ClawFlip;
      sleep(500);
      clawState = 1;
    } else {
      ClawFlip = !ClawFlip;
    }
  } else {
    ignoreRelease = false;
  }
}

void buttonRdown_pressed() {
  if (Controller1.ButtonR1.pressing()) {
    ignoreRelease = true;
    if (clawState == 1) {
      clawState = 13;
      ClawFlip = !ClawFlip;
      sleep(500);
      clawState = 1;
    } else {
      ClawFlip = !ClawFlip;
    }
  } else {
    ignoreRelease = false;
  }
}

void buttonRup_released() {
  if (!ignoreRelease) {
    if (!ClawFlip) {
      TopClaw = !TopClaw;
    } else {
      BottomClaw = !BottomClaw;
    }
  }
}

void buttonRdown_released() {
  if (!ignoreRelease) {
    if (!ClawFlip) {
      BottomClaw = !BottomClaw;
    } else {
      TopClaw = !TopClaw;
    }
  }
}

void buttonUP_pressed() { clawStatesActive = false; }

void buttonDOWN_pressed() { clawStatesActive = false; }

void buttonLEFT_pressed() { HangLock = !HangLock; }

void buttonRIGHT_pressed() { HangLock = !HangLock; }

void brain_pressed() {
  if (autonNumber > 6) {
    autonNumber = 1;
  } else {
    autonNumber += 1;
  }
}

void buttonX_pressed() { clawStatesActive = false; }

void buttonY_pressed() { MogoMech = !MogoMech; }

void buttonB_pressed() {
  if (autonNumber > 6) {
    autonNumber = 1;
  } else {
    autonNumber += 1;
  }
}

void buttonLup_pressed2() {}

void buttonLdown_pressed2() {}

void buttonRup_pressed2() {}

void buttonRdown_pressed2() {}

void buttonRdown_released2() {}

void buttonRup_released2() {}

void quals() {
  Inertial14.setRotation(-138 * headingMultiplier, deg);
  clawState = 3;
  sleep(300);
  driveDistance(25, 6, -138 * headingMultiplier);
  clawState = 2;
  sleep(250);
  BottomClaw = false;
  driveDistance(-50, 10, -138 * headingMultiplier);
  clawState = 3;
  driveTurn(40, 7 * headingMultiplier, 2);
  driveDistance(30, 21, 7 * headingMultiplier);
  driveDistance(30, 35, 100 * headingMultiplier);
  driveDistance(30, 15, 210 * headingMultiplier);
  MogoMech = true;
  if (headingMultiplier > 0) {
    clawState = 11;
  } else if (headingMultiplier < 0) {
    clawState = 1;
  }
  driveTorque = 70;
  driveTillStop(30, 165 * headingMultiplier);
  driveTorque = 100;
  TopClaw = true;
  BottomClaw = true;
  sleep(200);
  clawState = 12;
  driveDistance(-35, 10, 120 * headingMultiplier);
  clawState = 2;
  sleep(200);
  clawState = 21;
  driveDistance(-20, 35, 120 * headingMultiplier);
  clawState = 101;
  sleep(200);
  clawState = 1;
  MogoMech = false;
  sleep(750);
  clawState = 4;
  sleep(800);
  driveDistance(-40, 18, 120 * headingMultiplier);
}

void elims() {
  Inertial14.setRotation(20 * headingMultiplier, deg);
  clawState = 22;
  sleep(50);
  driveDistance(70, 50, 20 * headingMultiplier);
  clawState = 11;
  sleep(1000);
  driveDistance(-30, 13, 20 * headingMultiplier);
  BottomClaw = false;
  driveDistance(-30, 2, 20 * headingMultiplier);
  clawState = 4;
  driveDistance(-25, 12, 8 * headingMultiplier);
  driveTurn(40, -57 * headingMultiplier, 10);
  driveDistance(25, 12, -57 * headingMultiplier);
  driveDistance(40, 35, -255 * headingMultiplier);
  MogoMech = true;
  if (headingMultiplier > 0) {
    clawState = 102;
    armGoal = 15;
    clawGoal = 2;
  } else if (headingMultiplier < 0) {
    clawState = 15;
  }
  driveDistance(40, 15, -255 * headingMultiplier);
  driveTorque = 50;
  driveTillStop(30, -255 * headingMultiplier);
  sleep(100);
  TopClaw = true;
  BottomClaw = true;
  driveTorque = 100;
  sleep(400);
  clawState = 12;
  driveDistance(-40, 15, -225 * headingMultiplier);
  clawState = 21;
  sleep(1500);
  TopClaw = false;
  BottomClaw = false;
  clawState = 1;
  sleep(400);
  clawState = 3;
  sleep(400);
  MogoMech = false;
  driveDistance(-40, 5, -225 * headingMultiplier);
}

void skills() {
  Inertial14.setRotation(-180, deg);
  clawState = 3;
  sleep(250);
  driveDistance(25, 4, -180);
  clawState = 2;
  sleep(450);
  BottomClaw = false;
  driveDistance(-30, 3, -180);
  clawState = 15;
  driveTurn(60, -43, 10);
  driveDistance(30, 32, -43);
  BottomClaw = true;
  sleep(300);
  clawState = 3;
  driveTurn(60, -165, 3);
  driveDistance(25, 6, -180);
  clawState = 2;
  driveDistance(25, 2, -180);
  sleep(250);
  BottomClaw = false;
  clawState = 3;
  driveDistance(40, 25, -90);
  driveTorque = 50;
  driveTillStop(40, -100);
  driveTorque = 100;
  driveDistance(-40, 10, -45);
  // driveDistance(45, 15, -45);
  // clawState = 2;
  // driveDistance(30, 15, -100);
  // MogoMech = true;
  // clawState = 21;
  // sleep(1000);
  // clawState = 101;
  // sleep(500);
  // clawState = 3;
  // sleep(500);
  // clawState = 0;
  // driveTorque = 50;
  // driveTillStop(40, -135);
  // driveTorque = 100;
  // MogoMech = false;
  // driveDistance(-40, 15, -100);
}

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
    skills();
  } else if (autonNumber == 6) {
    PIDTest();
  }
}

void buttonA_pressed() {
  if (autonRunning) {
    autonRunning = false;
  } else {
    if (!autonHappened) {
      autonomous();
    }
    driveTorque = 100;
    driveHold = false;
  }
}

void usercontrol() {
  resetTimer();
  autonRunning = false;
  driveHold = false;
  driveTorque = 100;
  fieldControlState = 4;
  stopAll();
  Claw.spin(fwd);
  Arm1.spin(fwd);
  Arm2.spin(fwd);
  Claw.setStopping(hold);
  Arm1.setStopping(hold);
  Arm2.setStopping(hold);
  while (1) {
    sleep(10);
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

      axis1 = axis1 * axis1 / 100 * axis1 / abs(axis1);
      axis3 = axis3 * axis3 / 100 * axis3 / 100;

      leftSpeed = axis3 + axis1;
      // leftSpeedIncrement = (leftSpeed -
      // LFDrive.velocity(pct))/(10*fabs(tilt)); leftSpeed =
      // LFDrive.velocity(pct) + leftSpeedIncrement;

      rightSpeed = axis3 - axis1;
      // rightSpeedIncrement = (rightSpeed -
      // RFDrive.velocity(pct))/(10*fabs(tilt)); rightSpeed =
      // RFDrive.velocity(pct) + rightSpeedIncrement;

      driveTorque = 120 - 15 * tilt;

      if (LFDrive.velocity(pct) >= -20) {
        resetDrive();
      } else if ((clawState == 3 || clawState == 4) &&
                 (LFDrive.velocity(pct) < -20)) {
        if (fabs(avgDriveDistance) >= 6) {
          if (clawState == 3) {
            clawStateThree = true;
          }
          if (clawState == 4) {
            clawStateFour = true;
          }
          clawState = 2;
        }
      }
      if (clawStateThree && (LFDrive.velocity(pct) >= -10)) {
        clawState = 3;
        clawStateThree = false;
      }
      if (clawStateFour && (LFDrive.velocity(percent) >= -10)) {
        clawState = 4;
        clawStateFour = false;
      }

      // if (leftSpeed > 60) {
      //   leftSpeed = 60;
      // }
      // if (rightSpeed > 60) {
      //   rightSpeed = 60;
      // }
    }
  }
}

int main() {
  preAuton();
  task taskBrainScreen(brainScreenTask);
  task taskCntrlrScreen(controllerScreenTask);
  task taskSensors(sensorsTask);
  task taskDrive(driveTask);
  task taskClawSpeed(clawSpeedTask);
  task taskClawState(clawStatesTask);
  task taskOdometry(odometryTask);
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