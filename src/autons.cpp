#include "vex.h"
Func::Func() : headMod(), intakeSpeed(), armState() {}
/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants() {
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(6, .4, .03, 3, 15);
  chassis.set_swing_constants(10, .3, .001, 2, 15);
  // Each exit condition set is in the form of (settle_error, settle_time,
  // timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants() {
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}

/**
 * The expected behavior is to return to the start position.
 */

void drive_test() {
  default_constants();
  chassis.drive_distance(6);
  chassis.drive_distance(12);
  chassis.drive_distance(18);
  chassis.drive_distance(-36);
}

/**
 * The expected behavior is to return to the start angle, after making a
 * complete turn.
 */

void turn_test() {
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);
}

/**
 * Should swing in a fun S shape.
 */

void swing_test() {
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void full_test() {
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void odom_test() {
  chassis.set_coordinates(0, 0, 0);
  while (1) {
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5, 40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5, 60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 80, "ForwardTracker: %f",
                         chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5, 100, "SidewaysTracker: %f",
                         chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
}

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */

void tank_odom_test() {
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24, 24);
  chassis.drive_to_point(0, 0);
  chassis.turn_to_angle(0);
}

/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */

void holonomic_odom_test() {
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_pose(0, 18, 90);
  chassis.holonomic_drive_to_pose(18, 0, 180);
  chassis.holonomic_drive_to_pose(0, 18, 270);
  chassis.holonomic_drive_to_pose(0, 0, 0);
}

void redLeftNoWP() {
  chassis.set_coordinates(0, 0, 156);
  default_constants();
  chassis.drive_distance(-10.5);
  MogoMech = true;
  chassis.turn_to_angle(260);
  func.intakeSpeed = 100;
  chassis.drive_distance(10);
  task::sleep(250);
  chassis.drive_distance(-5);
  task::sleep(150);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(355);
  func.intakeSpeed = 100;
  chassis.drive_distance(6);
  task::sleep(350);
  chassis.drive_distance(-7);
  task::sleep(150);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(325);
  func.intakeSpeed = 100;
  chassis.drive_distance(8.5, 325);
  task::sleep(300);
  chassis.drive_distance(-6, 360);
  task::sleep(300);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(445);
  func.intakeSpeed = 100;
  chassis.drive_max_voltage = 6;
  chassis.drive_distance(20, 445);
}

void blueRightNoWP() {
  chassis.set_coordinates(0, 0, -158);
  default_constants();
  chassis.drive_distance(-11, -158);
  MogoMech = true;
  chassis.turn_to_angle(-260);
  func.intakeSpeed = 100;
  chassis.drive_distance(9, -275);
  task::sleep(250);
  chassis.drive_distance(-5, -270);
  task::sleep(150);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(-357);
  func.intakeSpeed = 100;
  chassis.drive_distance(5, -357);
  task::sleep(350);
  chassis.drive_distance(-7, -320);
  task::sleep(150);
  chassis.turn_to_angle(-320);
  func.intakeSpeed = 100;
  chassis.drive_distance(8.5, -320);
  task::sleep(300);
  chassis.drive_distance(-6, -360);
  task::sleep(300);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(-445);
  func.intakeSpeed = 100;
  chassis.drive_max_voltage = 6;
  chassis.drive_distance(20, -445);
}

void redLeftWP() {
  chassis.set_coordinates(0, 0, -90);
  default_constants();
  MogoMech = true;
  chassis.drive_distance(-5);
  chassis.turn_to_angle(0);
  chassis.drive_distance(-2);
  task::sleep(50);
  func.intakeSpeed = 100;
  task::sleep(200);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(140);
  MogoMech = false;
  chassis.drive_distance(-15);
  MogoMech = true;
  chassis.turn_to_angle(-90);
  func.intakeSpeed = 100;
  chassis.drive_distance(7);
  task::sleep(600);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(0);
  func.intakeSpeed = 100;
  chassis.drive_distance(5);
  task::sleep(200);
  chassis.drive_distance(-3);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(-26);
  func.intakeSpeed = 100;
  chassis.drive_distance(4);
  task::sleep(250);
  chassis.drive_distance(-5);
  chassis.turn_to_angle(90);
  chassis.drive_max_voltage = 12;
  chassis.drive_distance(50);
};

void blueRightWP() {
  chassis.set_coordinates(0, 0, 90);
  default_constants();
  MogoMech = true;
  chassis.drive_distance(-5.15);
  chassis.turn_to_angle(0);
  chassis.drive_distance(-2);
  task::sleep(50);
  func.intakeSpeed = 100;
  task::sleep(200);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(-143);
  MogoMech = false;
  chassis.drive_distance(-14);
  MogoMech = true;
  chassis.turn_to_angle(90);
  func.intakeSpeed = 100;
  chassis.drive_distance(7);
  task::sleep(650);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(0);
  func.intakeSpeed = 100;
  chassis.drive_distance(6.5);
  task::sleep(200);
  chassis.drive_distance(-4);
  chassis.turn_to_angle(27);
  chassis.drive_distance(5);
  task::sleep(250);
  chassis.drive_distance(-5);
  chassis.turn_max_voltage = 9;
  chassis.turn_to_angle(-90);
  chassis.drive_max_voltage = 12;
  chassis.drive_distance(50);
  func.intakeSpeed = 0;
};

void redGoalRush() {
  // red
  chassis.set_coordinates(0, 0, 180);
  default_constants();
  chassis.drive_max_voltage = 11;
  chassis.drive_distance(-9);
  chassis.turn_to_angle(153);
  chassis.drive_distance(-8.5);
  MogoMech = true;
  task::sleep(350);
  chassis.turn_to_angle(195);
  func.intakeSpeed = 100;
  chassis.drive_distance(4.5);
  //task::sleep(300);
  task::sleep(50);
  func.intakeSpeed = 0;
  MogoMech = false;
  //task::sleep(100);
  //func.intakeSpeed = 0;
  chassis.turn_to_angle(77);
  chassis.drive_distance(-9);
  MogoMech = true;
  task::sleep(200);
  chassis.turn_max_voltage = 6;
  chassis.turn_to_angle(-135);
  func.intakeSpeed = 100;
  // chassis.drive_distance(8);
  // task::sleep(2000);
  //   task::sleep(100);
  //   func.intakeSpeed = 0;
  //   IntakeLift = true;
  //   func.intakeSpeed = 100;
  //   chassis.drive_distance(12);
  //   IntakeLift = false;
  //   task::sleep(150);
  //   chassis.drive_distance(-1);
  //   func.intakeSpeed = 0;
  //   chassis.turn_to_angle(0);
  //   func.intakeSpeed = 100;
  //   task::sleep(400);
  //   chassis.drive_distance(7);
}

void blueGoalRush() {
  // blue
  chassis.set_coordinates(0, 0, -180);
  default_constants();
  chassis.drive_max_voltage = 11;
  chassis.drive_distance(-10);
  chassis.turn_to_angle(-153);
  chassis.drive_distance(-8);
  MogoMech = true;
  task::sleep(250);
  chassis.turn_to_angle(-195);
  func.intakeSpeed = 100;
  chassis.drive_distance(5);
  task::sleep(1000);
  MogoMech = false;
  task::sleep(150);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(-83);
  chassis.drive_distance(-10);
  MogoMech = true;
  task::sleep(200);
  func.intakeSpeed = 100;
  chassis.turn_max_voltage = 6;
  chassis.turn_to_angle(135);
  IntakeLift = true;
  func.intakeSpeed = 100;
  chassis.drive_distance(11);
  IntakeLift = false;
  task::sleep(300);
  chassis.drive_distance(-1);
  chassis.turn_to_angle(0);
  task::sleep(400);
  chassis.drive_distance(8);
}

void redRightWP() {}

void blueLeftWP() {
  chassis.set_coordinates(0, 0, 60);
  default_constants();
  IntakeLift = true;
  MogoMech = true;
  func.intakeSpeed = 100;
  task::sleep(300);
  func.intakeSpeed = 0;
  chassis.drive_distance(2);
  chassis.turn_to_angle(0);
  chassis.drive_distance(-5);
  task::sleep(50);
  func.intakeSpeed = 100;
  task::sleep(200);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(140);
  MogoMech = false;
  chassis.drive_distance(-15);
  MogoMech = true;
}