#include "vex.h"

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
// Inertial20           inertial      20              
// Claw                 motor         10              
// OdomX                rotation      8               
// OdomY                rotation      9               
// Arm                  motor_group   11, 12          
// ClawFlip             digital_out   A               
// ClawA                digital_out   B               
// ClawB                digital_out   C               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include <math.h>
#include <iostream> 
using namespace vex;
vex::competition Competition;

int fieldControlState = 0;
int axis1;
int axis2;
int axis3;
int axis4;

double gyro1;
int msecClock;

double avgDriveDistance;
double avgDriveSpeed;
int slowestDrive;
int fastestDrive;
int leftSpeed = 0;
int rightSpeed = 0;
int driveTorque = 100;
bool driveHold = false;

double wheelDiameter = 2.0; 
double wheelCircumference = wheelDiameter * M_PI;
double ticksPerRevolution = 360.0; // for v5 rotation sensor as they are sped
double x = 0.0;
double y = 0.0;
const int numReadings = 10;
double readingsX[numReadings];
double readingsY[numReadings]; // for sam: this just means that this variable is an array with a size of numReadings, which is 10.0, in the format of a double(same with readingsX). it is used in the odom function to find the average encoder value over 10 steps
int readIndex = 0;
double totalX = 0; 
double totalY = 0;
double averageX = 0; 
double averageY = 0;

int armSpeed;
int clawSpeed;
float clawState = 1;

bool autonRunning = false;
int autonNumber = 1;
bool autonHappened = false;


void sleep(int sleepmsec) { task::sleep(sleepmsec); }

void resetTimer() {
  Brain.resetTimer();
  sleep(5);
}

void resetGyro() {
  Inertial20.setRotation(0, deg);
  sleep(5);
}
void setGyro(int Heading) {
  Inertial20.setRotation(Heading, deg);
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
  fieldControlState = 0;
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(320, 200, "Pre Auton");

  resetDrive();
  sleep(100);
  Controller1.Screen.clearScreen();
  fieldControlState = 1;
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
      Brain.Screen.printAt(1, 210, "Auton: Close Side WP");
      Brain.Screen.setFillColor(red);
    } else if (autonNumber == 2) {
      Brain.Screen.printAt(1, 210, "Auton: Close Side Elims");
      Brain.Screen.setFillColor(blue);
    } else if (autonNumber == 3) {
      Brain.Screen.printAt(1, 210, "Auton: Far Side Safe");
      Brain.Screen.setFillColor("#008000");
    } else if (autonNumber == 4) {
      Brain.Screen.printAt(1, 210, "Auton: Far Side WP");
      Brain.Screen.setFillColor("#403e39");
    } else if (autonNumber == 5) {
      Brain.Screen.printAt(1, 210, "Auton: Far Side Elims");
      Brain.Screen.setFillColor(purple);
    } else if (autonNumber == 6) {
      Brain.Screen.printAt(1, 210, "Auton: Skills");
      Brain.Screen.setFillColor("#fc9e05");
    } else if (autonNumber == 7) {
      Brain.Screen.printAt(1, 210, "Auton: None");
      Brain.Screen.setFillColor(black);
    }
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

int controllerScreenTask() {
  Controller1.Screen.clearScreen();
  while (1) {
    sleep(50);

    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("gyro1: %3.0f  ", Inertial20.heading());

    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("Claw State: %1.0f", clawState);

    Controller1.Screen.setCursor(3, 4);
    Controller1.Screen.print("%2.0f ", LFDrive.temperature(fahrenheit) / 10);
    Controller1.Screen.setCursor(3, 7);
    Controller1.Screen.print("%2.0f ", RFDrive.temperature(fahrenheit) / 10);
    Controller1.Screen.setCursor(3, 10);
    Controller1.Screen.print("%2.0f ", LBDrive.temperature(fahrenheit) / 10);
    Controller1.Screen.setCursor(3, 13);
    Controller1.Screen.print("%2.0f ", RBDrive.temperature(fahrenheit) / 10);
    Controller1.Screen.setCursor(3, 16);
    Controller1.Screen.print("%2.0f ", LMDrive.temperature(fahrenheit) / 10);
    Controller1.Screen.setCursor(3, 19);
    Controller1.Screen.print("%2.0f ", RMDrive.temperature(fahrenheit) / 10);
    Controller1.Screen.setCursor(3, 22);
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
    gyro1 = Inertial20.rotation(deg);

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

    LFDrive.setMaxTorque(driveTorque, pct);
    LBDrive.setMaxTorque(driveTorque, pct);
    RFDrive.setMaxTorque(driveTorque, pct);
    RBDrive.setMaxTorque(driveTorque, pct);
    LMDrive.setMaxTorque(driveTorque, pct);
    RMDrive.setMaxTorque(driveTorque, pct);

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
  Arm.spin(forward);
  Claw.spin(forward);
  while (1) {
    sleep(5);
    Arm.setVelocity(armSpeed, pct);
    Claw.setVelocity(clawSpeed, pct);
  }
}

int clawStatesTask() {
  while(1){
    if (clawState == 1) {
      Arm.setVelocity(70, pct);
      Claw.setVelocity(70, pct);
      Arm.spinToPosition(30, deg);
      Claw.spinToPosition(0, deg);
    } else if (clawState == 2) {
      Arm.setVelocity(70, pct);
      Claw.setVelocity(70, pct);
      Arm.spinToPosition((45 * 7), deg);
      Claw.spinToPosition(0, deg);
    } else if (clawState == 3) {
      Arm.setVelocity(70, pct);
      Claw.setVelocity(70, pct);
      Arm.spinToPosition((90 * 7), deg);
      Claw.spinToPosition(0, deg);
    }
      sleep(5);
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

void autoTillStop(int Speed, double Heading) {
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

void turnTo(int Speed, int Heading, int Accuracy) {
  int integral = 0;
  int previousError = 0;
  double kP = 0.359;
  double kI = 0;
  double kD = 0.01;
  double lsp;
  double rsp;

  int newHeading = Heading + Accuracy - 1;
  while ((fabs(newHeading - gyro1) > Accuracy || (fabs(LFDrive.velocity(pct)) > 2.5)) && autonRunning) {
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
        double distance = sqrt(dx*dx + dy*dy);

        if (distance < 0.1) break; // Check early to avoid unnecessary calculations if already at target


        double target_angle = atan2(dy, dx);
        double turn_error = target_angle - gyro1;

        while (turn_error > M_PI) turn_error -= 2 * M_PI;
        while (turn_error < -M_PI) turn_error += 2 * M_PI;

        double fwd_error = distance;
        double turn_output = Kp_turn * turn_error + Ki_turn * turn_integral - Kd_turn * (turn_error - turn_prev_error);
        double fwd_output = Kp_fwd * fwd_error + Ki_fwd * fwd_integral - Kd_fwd * (fwd_error - fwd_prev_error);

        turn_integral += turn_error;
        turn_prev_error = turn_error;

        fwd_integral += fwd_error;
        fwd_prev_error = fwd_error;

        leftSpeed = speed * fwd_output + turn_output;
        rightSpeed =  speed * fwd_output - turn_output;

        sleep(5);
    }
}

void buttonLup_pressed() {armSpeed = 100;}

void buttonLdown_pressed() {armSpeed = -100;}

void buttonLup_released() {armSpeed = 0;}

void buttonLdown_released() {armSpeed = 0;}

void buttonRup_pressed() {clawSpeed = -100;}

void buttonRdown_pressed() {clawSpeed = 100;}

void buttonRup_released() {clawSpeed = 0;}

void buttonRdown_released() {clawSpeed = 0;}

void buttonUP_pressed() {
  if (clawState < 3) {
  clawState += 1;
  } else {
    clawState = 3;
  }
}

void buttonDOWN_pressed() {
  if (clawState > 1) {
    clawState -= 1;
  } else {
    clawState = 1;
  }
}

void buttonLEFT_pressed() {ClawFlip = !ClawFlip;}

void buttonRIGHT_pressed() {}

void brain_pressed() {}

void buttonX_pressed() {}



void buttonY_pressed() {ClawA = !ClawA;}

void buttonB_pressed() {ClawB = !ClawB;}

void buttonLup_pressed2() {}

void buttonLdown_pressed2() {}

void buttonRup_pressed2() {}

void buttonRdown_pressed2() {}

void buttonRdown_released2() {}

void buttonRup_released2() {}

void PIDTest() {

  }

void autonomous() {
  autonHappened = true;
  autonRunning = true;
  driveHold = true;
  if (autonNumber == 1) {
    PIDTest();
  } else if (autonNumber == 2) {

  } else if (autonNumber == 3) {

  } else if (autonNumber == 4) {
  
  } else if (autonNumber == 5) {

  } else if (autonNumber == 6) {

  }
}

void buttonA_pressed() {autonomous();}

void usercontrol() {
  resetTimer();
  autonRunning = false;
  driveHold = false;
  driveTorque = 100;
  fieldControlState = 4;
  stopAll();

  while (1) {
    sleep(10);
    if (autonRunning == false) {
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