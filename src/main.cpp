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
// Inertial21           inertial      21              
// ---- END VEXCODE CONFIGURED DEVICES ----
#include <cmath>

using namespace vex;
vex::competition Competition;

int field_control_state = 0;
int axis1;
int axis2;
int axis3;
int axis4;

double avg_drive_distance;
double avg_drive_speed;
double gyro1;
int msec_clock;
int slowest_drive;
int fastest_drive;

int left_speed = 0;
int right_speed = 0;
int drive_torque = 100;
bool drive_hold = false;

bool auton_running = false;
int auton_number = 1;
bool auton_happened = false;


void sleep(int sleepmsec) { task::sleep(sleepmsec); }

void reset_timer() {
  Brain.resetTimer();
  sleep(5);
}

void reset_gyro() {
  Inertial21.setRotation(0, deg);
  sleep(5);
}
void set_gyro(int Heading) {
  Inertial21.setRotation(Heading, deg);
  sleep(5);
}

void reset_drive() {
  LFDrive.resetPosition();
  RFDrive.resetPosition();
  LMDrive.resetPosition();
  RMDrive.resetPosition();
  LBDrive.resetPosition();
  RBDrive.resetPosition();
  sleep(5);
}

void stop_drive() {
  left_speed = 0;
  right_speed = 0;
  sleep(5);
}

void stop_all() {
  LFDrive.stop();
  RFDrive.stop();
  LMDrive.stop();
  RMDrive.stop();
  LBDrive.stop();
  RBDrive.stop();
  sleep(5);
}

void pre_auton() {
  field_control_state = 0;
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(320, 200, "Pre Auton");

  reset_drive();
  sleep(100);
  Controller1.Screen.clearScreen();
  field_control_state = 1;
}

int brain_screen_task() {
  while (1) {
    sleep(100);
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(1, 20, "LFMotor: %5.2f   ", gyro1);
    Brain.Screen.printAt(188, 20, "RFMotor: %5.2f   ", gyro1);
    Brain.Screen.printAt(1, 40, "LTMotor: %5.2f   ", gyro1);
    Brain.Screen.printAt(188, 40, "RTMotor: %5.2f   ", gyro1);
    Brain.Screen.printAt(1, 60, "Avg Motor Dist: %4.2f   ", avg_drive_distance);
    Brain.Screen.printAt(1, 100, "gyro1: %5.2f    ", gyro1);
    Brain.Screen.printAt(1, 140, "msec_clock: %d    ", msec_clock);
    Brain.Screen.printAt(1, 170, "       ");
    Brain.Screen.printAt(370, 20, "Axis1: %d", axis1);
    Brain.Screen.printAt(370, 40, "Axis2: %d", axis2);
    Brain.Screen.printAt(370, 60, "Axis3: %d", axis3);
    Brain.Screen.printAt(370, 80, "Axis4: %d", axis4);
    Brain.Screen.printAt(320, 110, "Reflect: %3.2f  ");
    Brain.Screen.printAt(320, 150, "PotR: %3.2f   ");

    if (auton_number == 1) {
      Brain.Screen.printAt(1, 210, "Auton: Close Side WP");
      Brain.Screen.setFillColor(red);
    } else if (auton_number == 2) {
      Brain.Screen.printAt(1, 210, "Auton: Close Side Elims");
      Brain.Screen.setFillColor(blue);
    } else if (auton_number == 3) {
      Brain.Screen.printAt(1, 210, "Auton: Far Side Safe");
      Brain.Screen.setFillColor("#008000");
    } else if (auton_number == 4) {
      Brain.Screen.printAt(1, 210, "Auton: Far Side WP");
      Brain.Screen.setFillColor("#403e39");
    } else if (auton_number == 5) {
      Brain.Screen.printAt(1, 210, "Auton: Far Side Elims");
      Brain.Screen.setFillColor(purple);
    } else if (auton_number == 6) {
      Brain.Screen.printAt(1, 210, "Auton: Skills");
      Brain.Screen.setFillColor("#fc9e05");
    } else if (auton_number == 7) {
      Brain.Screen.printAt(1, 210, "Auton: None");
      Brain.Screen.setFillColor(black);
    }
  }

  if (field_control_state == 0) {
    Brain.Screen.printAt(320, 200, "Pre Auton");
  }
  if (field_control_state == 1) {
    Brain.Screen.printAt(320, 200, "Pre Auton Done");
  }
  if (field_control_state == 2) {
    Brain.Screen.printAt(320, 200, "Autonomous");
  }
  if (field_control_state == 3) {
    Brain.Screen.printAt(320, 200, "Autonomous Done");
  }
  if (field_control_state == 4) {
    Brain.Screen.printAt(320, 200, "Driver");
  }
}

int controller_screen_task() {
  Controller1.Screen.clearScreen();
  while (1) {
    sleep(50);

    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("gyro1: %3.0f  ", Inertial21.heading());

    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.clearLine(2);

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

int sensors_task() {
  int x = 100;
  while (1) {
    sleep(5);
    // GET MOTOR ENCODERS AND SCALE THEM TO DISTANCE IN INCHES(450 RPM)
    avg_drive_distance = (LFDrive.position(deg) + RMDrive.position(deg)) * 0.0120;

    // GET AVERAGE MOTOR SPEED PERCENTAGE
    avg_drive_speed = (LFDrive.velocity(pct) + RMDrive.velocity(pct)) * .5;

    // GET gyro1 VALUE
    gyro1 = Inertial21.rotation(deg);

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
    slowest_drive = abs(x);

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
    fastest_drive = abs(x);

    msec_clock = Brain.timer(msec);
  }
}

int drive_task() {
  while (1) {
    if (drive_hold) {
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

    LFDrive.setMaxTorque(drive_torque, pct);
    LBDrive.setMaxTorque(drive_torque, pct);
    RFDrive.setMaxTorque(drive_torque, pct);
    RBDrive.setMaxTorque(drive_torque, pct);
    LMDrive.setMaxTorque(drive_torque, pct);
    RMDrive.setMaxTorque(drive_torque, pct);

    LFDrive.spin(fwd, left_speed, pct);
    LBDrive.spin(fwd, left_speed, pct);
    LMDrive.spin(fwd, left_speed, pct);
    RFDrive.spin(fwd, right_speed, pct);
    RBDrive.spin(fwd, right_speed, pct);
    RMDrive.spin(fwd, right_speed, pct);

    sleep(6);
  }
}

void drive_distance(int Speed, double Distance, double Heading) {
  double kP = 0.4;
  double kI = 0.06;
  double kD = 0.0;
  double integral = 0;
  double previous_error = 0;

  reset_drive();
  sleep(10);
  double RightTurnDiff;
  while ((fabs(avg_drive_distance) < Distance) && auton_running) {
    double error = Heading - gyro1;
    integral += error;
    double derivative = error - previous_error;
    RightTurnDiff = kP * error + kI * integral + kD * derivative;
    left_speed = Speed + RightTurnDiff;
    right_speed = Speed - RightTurnDiff;
    previous_error = error;
    sleep(10);
  }
  stop_drive();
}

void drive_till_stop(int Speed, double Heading) {
  reset_drive();
  reset_timer();
  sleep(10);
  int RightTurnDiff;
  while ((slowest_drive > 3 || msec_clock < 500) && auton_running) {
    RightTurnDiff = (Heading - gyro1) * .65;
    left_speed = Speed + RightTurnDiff;
    right_speed = Speed - RightTurnDiff;
    sleep(10);
  }
  stop_drive();
}

  int integral = 0;
  int previous_error = 0;
  double kP = 0.36;
  double kI = 0;
  double kD = 0;
void turn(int Speed, int Heading, int Accuracy) {
  double lsp;
  double rsp;

  int new_heading = Heading + Accuracy - 1;
  while ((fabs(new_heading - gyro1) > Accuracy || (fabs(LFDrive.velocity(pct)) > 2.5)) && auton_running) {
    double error = new_heading - gyro1;
    integral += error;
    double derivative_turn = error - previous_error;
    double output = kP * error + kI * integral + kD * derivative_turn;
    lsp = +output;
    if (fabs(lsp) < 2) {
      lsp = 2 * fabs(lsp) / lsp;
    }
    left_speed = lsp;
    rsp = -output;
    if (fabs(rsp) < 2) {
      rsp = 2 * fabs(rsp) / rsp;
    }
    right_speed = rsp;
    previous_error = error;
    sleep(5);
  }

  stop_drive();
  // integral = 0;
  // previous_error = 0;
  sleep(10);
}

void buttonLup_pressed() {
}

void buttonLdown_pressed() {}

void buttonLup_released() {}

void buttonLdown_released() {}

void buttonRup_pressed() {}

void buttonRdown_pressed() {}

void buttonRup_released() {}

void buttonRdown_released() {}

void buttonUP_pressed() {}

void buttonDOWN_pressed() {}

void buttonRIGHT_pressed() {}

void buttonLEFT_pressed() {}

void brain_pressed() {}

void buttonX_pressed() {}

void buttonA_pressed() {}

void buttonY_pressed() {}

void buttonB_pressed() {}

void buttonLup_pressed2() {}

void buttonLdown_pressed2() {}

void buttonRup_pressed2() {}

void buttonRdown_pressed2() {}

void buttonRdown_released2() {}

void buttonRup_released2() {}

void autonomous() {
  auton_happened = true;
  auton_running = true;
  drive_hold = true;
  if (auton_number == 1) {

  } else if (auton_number == 2) {

  } else if (auton_number == 3) {

  } else if (auton_number == 4) {
  
  } else if (auton_number == 5) {

  } else if (auton_number == 6) {

  }
}

void usercontrol() {
  reset_timer();
  auton_running = false;
  drive_hold = false;
  drive_torque = 100;
  field_control_state = 4;
  stop_all();

  while (1) {
    sleep(10);
    if (auton_running == false) {
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

      left_speed = axis3 + axis1;
      right_speed = axis3 - axis1;
    }
  }
}

int main() {
  pre_auton();
  task taskBrainScreen(brain_screen_task);
  task taskCntrlrScreen(controller_screen_task);
  task taskSensors(sensors_task);
  task taskDrive(drive_task);
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