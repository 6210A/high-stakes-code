#include "vex.h"
Func::Func() : headMod(), conveyorSpeed(), rollerSpeed(), armState() {}
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
  chassis.set_turn_constants(6, .4, .03, 4, 15);
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
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.drive_distance(10);
  task::sleep(250);
  chassis.drive_distance(-5);
  task::sleep(150);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  chassis.turn_to_angle(355);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.drive_distance(6);
  task::sleep(350);
  chassis.drive_distance(-7);
  task::sleep(150);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  chassis.turn_to_angle(328);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.drive_distance(9.5, 328);
  task::sleep(300);
  chassis.drive_distance(-6, 360);
  task::sleep(300);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  chassis.turn_to_angle(445);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.drive_max_voltage = 6;
  chassis.drive_distance(20, 445);
}

void blueRightNoWP() {
  chassis.set_coordinates(0, 0, -158);
  default_constants();
  chassis.drive_distance(-11, -158);
  MogoMech = true;
  chassis.turn_to_angle(-270);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.drive_distance(9, -270);
  task::sleep(250);
  chassis.drive_distance(-5, -270);
  task::sleep(150);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  chassis.turn_to_angle(-345);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.drive_distance(5, -345);
  task::sleep(350);
  chassis.drive_distance(-7, -320);
  task::sleep(150);
  chassis.turn_to_angle(-320);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.drive_distance(8.5, -320);
  task::sleep(300);
  chassis.drive_distance(-8, -360);
  task::sleep(300);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  chassis.turn_to_angle(-445);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.drive_max_voltage = 6;
  chassis.drive_distance(20, -445);
}

void redLeftWP2() {
  chassis.set_coordinates(0, 0, 145);
  default_constants();
  chassis.set_drive_constants(11, 1.9, 0, 10, 0);
  func.conveyorSpeed = -100;
  task::sleep(150);
  func.armState = 2;
  task::sleep(1300);
  func.armState = 0;
  func.conveyorSpeed = 0;
  chassis.turn_to_angle(161);
  chassis.drive_distance(-25);
  chassis.drive_max_voltage = 8;
  chassis.drive_distance(-10);
  MogoMech = true;
  chassis.turn_to_angle(315);
  func.conveyorSpeed = 100;
  func.rollerSpeed = 100;
  chassis.drive_distance(21);
  chassis.turn_to_angle(275);
  chassis.drive_distance(15);
  task::sleep(200);
  chassis.drive_distance(-17);
  chassis.turn_to_angle(210);
  chassis.drive_distance(13);
  task::sleep(400);
  chassis.turn_to_angle(80);
  chassis.drive_max_voltage = 8;
  chassis.drive_distance(30);
}

void redLeftWP() {
  chassis.set_coordinates(0, 0, -90);
  default_constants();
  chassis.drive_max_voltage = 11;
  chassis.turn_max_voltage = 9;
  func.conveyorSpeed = -100;
  task::sleep(150);
  func.conveyorSpeed = 0;
  chassis.drive_distance(-14.25);
  chassis.turn_to_angle(0);
  chassis.drive_distance(-3);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  task::sleep(600);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  chassis.drive_distance(2);
  chassis.turn_to_angle(141);
  chassis.drive_distance(-30);
  chassis.drive_max_voltage = 6;
  chassis.drive_distance(-9);
  MogoMech = true;
  task::sleep(150);
  chassis.drive_max_voltage = 11;
  chassis.turn_to_angle(-90);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.drive_distance(30);
  // task::sleep(400);
  chassis.turn_to_angle(-5);//
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.drive_distance(15.5);
  task::sleep(200);
  chassis.drive_distance(-11);
  chassis.turn_to_angle(24);//
  chassis.drive_distance(12.5);
  task::sleep(500);
  chassis.drive_distance(-12);
  chassis.turn_max_voltage = 10;
  chassis.turn_to_angle(88);
  chassis.drive_distance(36);
  chassis.drive_max_voltage = 3;
  chassis.drive_distance(2);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
};

void blueRightWP2() {
  chassis.set_coordinates(0, 0, -145);
  default_constants();
  chassis.set_drive_constants(11, 1.9, 0, 10, 0);
  func.conveyorSpeed = -100;
  task::sleep(150);
  func.armState = 2;
  task::sleep(1300);
  func.armState = 0;
  func.conveyorSpeed = 0;
  chassis.turn_to_angle(-161);
  chassis.drive_distance(-25);
  chassis.drive_max_voltage = 8;
  chassis.drive_distance(-10);
  MogoMech = true;
  chassis.turn_to_angle(-315);
  func.conveyorSpeed = 100;
  func.rollerSpeed = 100;
  chassis.drive_distance(21);
  chassis.turn_to_angle(-275);
  chassis.drive_distance(15);
  task::sleep(200);
  chassis.drive_distance(-17);
  chassis.turn_to_angle(-210);
  chassis.drive_distance(13);
  task::sleep(400);
  chassis.turn_to_angle(-80);
  chassis.drive_max_voltage = 8;
  chassis.drive_distance(30);
}

void blueRightWP() {
  chassis.set_coordinates(0, 0, 90);
  default_constants();
  chassis.drive_max_voltage = 11;
  chassis.turn_max_voltage = 9;
  func.conveyorSpeed = -100;
  task::sleep(150);
  func.conveyorSpeed = 0;
  chassis.drive_distance(-14.25);
  chassis.turn_to_angle(0);
  chassis.drive_distance(-3);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  task::sleep(600);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  chassis.drive_distance(2);
  chassis.turn_to_angle(-141);
  chassis.drive_distance(-30);
  chassis.drive_max_voltage = 6;
  chassis.drive_distance(-9);
  MogoMech = true;
  task::sleep(150);
  chassis.drive_max_voltage = 11;
  chassis.turn_to_angle(90);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.drive_distance(30);
  // task::sleep(400);
  chassis.turn_to_angle(-5);//
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.drive_distance(16);
  task::sleep(200);
  chassis.drive_distance(-11);
  chassis.turn_to_angle(-24);//
  chassis.drive_distance(13);
  task::sleep(500);
  chassis.drive_distance(-12);
  chassis.turn_max_voltage = 10;
  chassis.turn_to_angle(-88);
  chassis.drive_distance(36);
  chassis.drive_max_voltage = 3;
  chassis.drive_distance(3);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
};

void redGoalRush() {
  chassis.set_coordinates(0, 0, 0);
  default_constants();
  chassis.drive_max_voltage = 12;
  func.conveyorSpeed = -100;
  task::sleep(150);
  func.conveyorSpeed = 0;
  chassis.drive_distance(30);
  chassis.turn_max_voltage = 11;
  chassis.drive_distance(11, -20);
  Doinker = true;
  task::sleep(300);
  chassis.drive_max_voltage = 10;
  chassis.drive_distance(-10, -10);
  Doinker = false;
  task::sleep(100);
  chassis.drive_distance(-3, -10);
  chassis.drive_max_voltage = 12;
  chassis.drive_distance(-9, 0);
  func.rollerSpeed = 100;
  chassis.turn_to_angle(-50);
  chassis.drive_distance(10);
  task::sleep(50);
  chassis.turn_to_angle(-175);
  chassis.drive_distance(-10);
  MogoMech = true;
  func.conveyorSpeed = 80;
  chassis.drive_distance(-3);
  chassis.drive_distance(2);
  func.conveyorSpeed = 0;
  func.rollerSpeed = 0;
  MogoMech = false;
  task::sleep(100);
  chassis.turn_max_voltage = 8;
  chassis.drive_distance(24);
  chassis.turn_to_angle(-240);
  chassis.turn_max_voltage = 11;
  chassis.drive_distance(-22);
  chassis.drive_max_voltage = 8;
  chassis.drive_distance(-5);
  MogoMech = true;
  chassis.drive_max_voltage = 12;
  func.conveyorSpeed = 100;
  IntakeLift = true;
  func.rollerSpeed = 100;
  chassis.turn_to_angle(-135);
  chassis.drive_distance(25);
  IntakeLift = false;
  chassis.turn_to_angle(-35);
  chassis.drive_max_voltage = 8;
  chassis.drive_distance(10);
  /* chassis.turn_to_angle(153);
  chassis.drive_distance(-8);
  MogoMech = true;
  task::sleep(350);
  chassis.turn_to_angle(185);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.drive_distance(5);
  task::sleep(500);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  MogoMech = false;
  // task::sleep(100);
  // func.conveyorSpeed = 0; func.rollerSpeed = 0;
  chassis.turn_to_angle(90);
  chassis.drive_distance(-9);
  MogoMech = true;
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  task::sleep(200);
  chassis.turn_max_voltage = 6;
  chassis.turn_to_angle(-135);
  // chassis.turn_to_angle(-90);
  // func.conveyorSpeed = 100; func.rollerSpeed = 100;
  // chassis.drive_distance(14);
  // chassis.drive_distance(8);
  // task::sleep(2000);
  //   task::sleep(100);
  //   func.conveyorSpeed = 0; func.rollerSpeed = 0;
  IntakeLift = true;
  // func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.drive_distance(12);
  IntakeLift = false;
  task::sleep(150);
  chassis.drive_distance(-1);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  chassis.turn_to_angle(0);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  task::sleep(400);
  chassis.drive_distance(7);*/

}

void blueGoalRush() {
  // blue
  chassis.set_coordinates(0, 0, -180);
  default_constants();
  //
  chassis.drive_max_voltage = 11;
  chassis.drive_distance(-10);
  chassis.turn_to_angle(-153);
  chassis.drive_distance(-7.75);
  MogoMech = true;
  task::sleep(250);
 // chassis.drive_max_voltage = 11;
  chassis.turn_to_angle(-195);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.drive_distance(5);
  task::sleep(1000);
  MogoMech = false;
  task::sleep(150);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  chassis.turn_to_angle(-83);
  chassis.drive_distance(-10);
  MogoMech = true;
  task::sleep(200);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.turn_max_voltage = 6;
  chassis.turn_to_angle(135);
  IntakeLift = true;
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.drive_distance(11);
  IntakeLift = false;
  task::sleep(300);
  chassis.drive_distance(-1);
  // chassis.turn_to_angle(0);
  // task::sleep(400);
  // chassis.drive_distance(8);
  // elims
  // chassis.turn_to_angle(-110);
  //
}

void redRightWP() {}

void blueLeftWP() {
  chassis.set_coordinates(0, 0, 60);
  default_constants();
  IntakeLift = true;
  MogoMech = true;
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  task::sleep(300);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  chassis.drive_distance(2);
  chassis.turn_to_angle(0);
  chassis.drive_distance(-5);
  task::sleep(50);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  task::sleep(200);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  chassis.turn_to_angle(140);
  MogoMech = false;
  chassis.drive_distance(-15);
  MogoMech = true;
}

void skillsAuton() {
  chassis.set_coordinates(0, 0, 0);
  chassis.heading_max_voltage = 8;
  default_constants();
  MogoMech = true;
  chassis.drive_distance(-0.5);
  task::sleep(50);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  task::sleep(200);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  chassis.drive_distance(5.5);
  MogoMech = false;
  chassis.turn_to_angle(90);
  chassis.drive_distance(-9);
  MogoMech = true;
  task::sleep(75);
  chassis.turn_to_angle(-90);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.drive_distance(12);
  task::sleep(200);
  chassis.turn_to_angle(15);
  chassis.drive_distance(-4);
  MogoMech = false;
  task::sleep(75);
  chassis.drive_distance(3.5);
  chassis.turn_to_angle(-92);
  chassis.drive_max_voltage = 9;
  chassis.drive_distance(-29.5);
  MogoMech = true;
  chassis.turn_to_angle(90);
  chassis.drive_distance(11.5);
  task::sleep(200);
  chassis.turn_to_angle(-18);
  chassis.drive_distance(-3.75);
  MogoMech = false;
  task::sleep(75);
  chassis.drive_distance(3.75);
  func.armState = 1;
  chassis.turn_to_angle(0);
  chassis.drive_distance(16.5);
  chassis.turn_to_angle(90);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  chassis.drive_distance(2.65);
  func.armState = 2;
  func.conveyorSpeed = -100; func.rollerSpeed = -100;
  task::sleep(800);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  func.armState = 0;
  task::sleep(50);
  chassis.turn_to_angle(-25);
  task::sleep(300);
  func.conveyorSpeed = 30; func.rollerSpeed = 30;
  chassis.drive_distance(10);
  chassis.turn_to_angle(-90);
  func.conveyorSpeed = 40; func.rollerSpeed = 40;
  chassis.drive_distance(10);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  chassis.turn_to_angle(132);
  chassis.drive_distance(-13);
  MogoMech = true;
  task::sleep(50);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  chassis.turn_to_angle(224);
  chassis.drive_max_voltage = 9;
  chassis.drive_distance(23);
  chassis.drive_max_voltage = 10;
  func.armState = 1;
  chassis.turn_to_angle(270);
  chassis.drive_distance(4.25);
  func.conveyorSpeed = -100; func.rollerSpeed = -100;
  func.armState = 2;
  task::sleep(800);
  func.armState = 0;
  task::sleep(75);
  func.conveyorSpeed = 45; func.rollerSpeed = 45;
  chassis.turn_to_angle(12);
  MogoMech = false;
  chassis.drive_distance(7);
  func.conveyorSpeed = 27; func.rollerSpeed = 27;
  chassis.turn_to_angle(53);
  chassis.drive_distance(23);
  chassis.turn_to_angle(180);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  MogoMech = true;
  chassis.drive_distance(-2);
  task::sleep(50);
  func.conveyorSpeed = 100; func.rollerSpeed = 100;
  task::sleep(200);
  func.conveyorSpeed = 0; func.rollerSpeed = 0;
  task::sleep(50);
  chassis.turn_to_angle(-90);
  chassis.drive_distance(20);
  chassis.turn_to_angle(90);
  chassis.drive_distance(555);
}

void red2On1() {
  default_constants();
  chassis.set_coordinates(0, 0, 180);
  chassis.drive_distance(-10);
}