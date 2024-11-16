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
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

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
  chassis.set_coordinates(0, 0, 154);
  default_constants();
  chassis.drive_distance(-10.5, 153);
  MogoMech = true;
  chassis.turn_to_angle(260);
  func.intakeSpeed = 100;
  chassis.drive_distance(9, 260);
  task::sleep(250);
  chassis.drive_distance(-5, 270);
  task::sleep(150);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(340);
  func.intakeSpeed = 100;
  chassis.drive_distance(6, 340);
  task::sleep(350);
  chassis.drive_distance(-7, 320);
  task::sleep(150);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(270);
  func.intakeSpeed = 100;
  chassis.drive_distance(8, 270);
  task::sleep(150);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(360);
  func.intakeSpeed = 100;
  chassis.drive_distance(6, 360);
  task::sleep(350);
  chassis.drive_distance(-7, 360);
  task::sleep(300);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(445);
  func.intakeSpeed = 100;
  chassis.drive_max_voltage = 6;
  chassis.drive_distance(20, 445);
}

void blueRightNoWP() {
  chassis.set_coordinates(0, 0, -154);
  default_constants();
  chassis.drive_distance(-11, -153);
  MogoMech = true;
  chassis.turn_to_angle(-260);
  func.intakeSpeed = 100;
  chassis.drive_distance(9, -260);
  task::sleep(250);
  chassis.drive_distance(-5, -270);
  task::sleep(150);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(-340);
  func.intakeSpeed = 100;
  chassis.drive_distance(6, -340);
  task::sleep(350);
  chassis.drive_distance(-7, -320);
  task::sleep(150);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(-270);
  func.intakeSpeed = 100;
  chassis.drive_distance(8, -270);
  task::sleep(150);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(-360);
  func.intakeSpeed = 100;
  chassis.drive_distance(6, -360);
  task::sleep(350);
  chassis.drive_distance(-7, -360);
  task::sleep(300);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(-445);
  func.intakeSpeed = 100;
  chassis.drive_max_voltage = 11;
  chassis.drive_distance(20, -445);
}

void redLeftWP(){};

void blueRightWP() {
  // func.intakeSpeed = 100;
  chassis.set_coordinates(0, 0, -61);
  default_constants();
  chassis.turn_max_voltage = 6;
  MogoMech = true;
  func.intakeSpeed = 20;
  chassis.drive_distance(1);
  task::sleep(100);
  chassis.turn_to_angle(15);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(37);
  chassis.turn_max_voltage = 9;
  chassis.turn_max_voltage = 11;
  chassis.drive_distance(-6, 37);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(0);
  func.intakeSpeed = -100;
  task::sleep(100);
  func.intakeSpeed = 0;
  chassis.drive_distance(-2.5, 0);
  func.intakeSpeed = 100;
  task::sleep(300);
  func.intakeSpeed = 0;
  chassis.drive_distance(4, 0);
  chassis.turn_to_angle(-140);
  MogoMech = false;
  chassis.drive_max_voltage = 8;
  chassis.drive_distance(-16);
  MogoMech = true;
  task::sleep(500);
  chassis.turn_to_angle(93);
  func.intakeSpeed = 100;
  chassis.drive_max_voltage = 11;
  chassis.drive_distance(10);
  chassis.turn_to_angle(260);
  chassis.drive_distance(15);

  // chassis.drive_distance(-44, 160);
  // MogoMech = true;
  // task::sleep(50);
  // chassis.drive_distance(2, 160);
  // func.intakeSpeed = 100;
  // task::sleep(300);
  // func.intakeSpeed = 0;
  // chassis.turn_to_angle(-90);
  // func.intakeSpeed = 100;
  // chassis.drive_distance(18, -90);
  // func.intakeSpeed = 100;*/
};

void blueGoalRush() {
  chassis.set_coordinates(0, 0, 180);
  default_constants();
  chassis.drive_max_voltage = 11;
  chassis.drive_distance(-10);
  chassis.turn_to_angle(153);
  chassis.drive_distance(-9);
  MogoMech = true;
  task::sleep(250);
  chassis.turn_to_angle(195);
  func.intakeSpeed = 100;
  chassis.drive_distance(3.5);
  task::sleep(300);
  MogoMech = false;
  task::sleep(150);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(70);
  chassis.drive_distance(-9);
  MogoMech = true;
  task::sleep(200);
  chassis.turn_max_voltage = 6;
  chassis.turn_to_angle(230);
  IntakeLift = true;
  func.intakeSpeed = 100;
  chassis.drive_distance(12);
  task::sleep(150);
  chassis.drive_distance(-1);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(0);
  func.intakeSpeed = 100;
  task::sleep(400);
  chassis.drive_distance(6);
}

void redGoalRush() {
  chassis.set_coordinates(0, 0, -180);
  default_constants();
  chassis.drive_max_voltage = 11;
  chassis.drive_distance(-10);
  chassis.turn_to_angle(-153);
  chassis.drive_distance(-9);
  MogoMech = true;
  task::sleep(250);
  chassis.turn_to_angle(-195);
  func.intakeSpeed = 100;
  chassis.drive_distance(3.5);
  task::sleep(300);
  MogoMech = false;
  task::sleep(150);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(-70);
  chassis.drive_distance(-9);
  MogoMech = true;
  task::sleep(200);
  chassis.turn_max_voltage = 6;
  chassis.turn_to_angle(-230);
  IntakeLift = true;
  func.intakeSpeed = 100;
  chassis.drive_distance(12);
  task::sleep(150);
  chassis.drive_distance(-1);
  func.intakeSpeed = 0;
  chassis.turn_to_angle(0);
  func.intakeSpeed = 100;
  task::sleep(400);
  chassis.drive_distance(6);
}