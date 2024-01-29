#include "autons.hpp"
#include "lemlib/api.hpp"
#include "main.h"
#include "math.h"
#include "pros/rtos.hpp"

const int DRIVE_SPEED =
    100; // This is 110/127 (around 87% of max speed).  We don't suggest making
         // this 127. If this is 127 and the robot tries to heading correct,
         // it's only correcting by making one side slower.  When this is 87%,
         // it's correcting by making one side faster and one side slower,
         // giving better heading correction.
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;
const int WING_ACTIVATE_DELAY = 250;

int timeout = 1200;

/// @brief do defensive/left side
void defensive_side() {
  // start pos
  chassis.setPose(-32, -56, 0);
  int level = -12;
  // go forward
  chassis.moveToPoint(-32, level, timeout);

  // turn left and release ball
  chassis.turnToPoint(-600, level, timeout);
  intake.move_velocity(-600);
  pros::delay(WING_ACTIVATE_DELAY);
  chassis.turnToPoint(600, level, timeout);

  /* chassis.turnToPoint(600,-7,timeout);*/
  chassis.waitUntilDone();

  // activate wings
  wings.set_value(true);

  // quick tap the ball
  chassis.moveToPoint(-40, level, timeout, {.forwards = false}, DRIVE_SPEED);
  chassis.moveToPoint(-32, level, timeout);

  // retract wings
  wings.set_value(false);

  // go to match load area
  chassis.moveToPoint(-60, -54, timeout + 500);

  // take away match load
  chassis.turnToPoint(-60 + 600, -56 - 600, timeout, SWING_SPEED);
  wings.set_value(true);
  chassis.turnToPoint(-60 + 600, -54 + 600, timeout, true, DRIVE_SPEED, false);
  wings.set_value(false);

  // touch elevation bar
  chassis.moveToPoint(-3, -100, timeout);
}
void defensive_side_mess_up() {
  int level = -10;
  /*/ take out triball
  chassis.setPose(-49, -52, 105);
  wings.set_value(true);
  pros::delay(WING_ACTIVATE_DELAY);
  chassis.turnToPoint(10e4, 10e4, 1200, true, 90);

  // go to middle
  wings.set_value(false);
  chassis.moveToPoint(-24, -12, 2000);

  // push triball
  chassis.turnToPoint(-60, -12, 1200, false, 90);
  wings.set_value(false);
  chassis.moveToPoint(-446, -12, 1200, false, 110);

  // go to bar
  chassis.moveToPoint(-24, -24, 1200, true, 110);
  chassis.turnToPoint(0, -48, 1200, false, 90);
  chassis.moveToPoint(-4, -44, 1200, false, 110);//*/

  // go to middle
  chassis.setPose(-48 - 7.5, -60 + 7.25, 0);
  chassis.moveToPoint(chassis.getPose().x, level, 1200,
                      {.maxSpeed = DRIVE_SPEED});

  /*/ mess up
  chassis.turnToPoint(60, level, 800, false, SWING_SPEED);
  chassis.waitUntilDone();
  wings.set_value(true);
  pros::delay(WING_ACTIVATE_DELAY);
  chassis.moveToPoint(-8, level, 1200, false, DRIVE_SPEED);//*/

  // put red triball in goal
  chassis.turnToPoint(-60, level, 800, true, SWING_SPEED);
  chassis.moveToPoint(-48 - 2, level, 1200, {.maxSpeed = 60});

  /*/ go back
  chassis.turnToPoint(60, level, 800, false, SWING_SPEED);
  chassis.moveToPoint(-24, -24, 1200, false, DRIVE_SPEED);
  chassis.turnToPoint(-50,-50, 1200, false, SWING_SPEED);
  chassis.moveToPoint(-50,-50, 1800, false, DRIVE_SPEED);
  // actually go back
  chassis.moveToPoint(-24,-24, 900, true, DRIVE_SPEED);
  chassis.moveToPoint(-50,-50, 1800, false, DRIVE_SPEED);

  // get match load out
  chassis.waitUntilDone();
  wings.set_value(true);
  pros::delay(WING_ACTIVATE_DELAY);
  chassis.moveToPose(-36, -60, -90, 2400, {.forwards = false, .maxSpeed = 60});

  // touch bar
  chassis.turnToPoint(0, -60, 1200, true);
  chassis.moveToPoint(-4, -60, 800, true, 60);//*/
}
ASSET(returnbackdefense_txt)
void defensive_side_safe() {
  // start pos
  chassis.setPose(-49, -52, 90);
  wings.set_value(true);
  pros::delay(WING_ACTIVATE_DELAY);
  chassis.moveToPoint(-36, -60, 8000);
  chassis.waitUntil(15);
  wings.set_value(false);
  chassis.moveToPoint(-11, -60, 8000);
}

/// @brief do offensive/right side
void offensive_side() {
  // start pos
  chassis.setPose(36, -56, 0);

  // go to first triball
  chassis.moveToPoint(32, -32, timeout);
  chassis.turnToPoint(48, 0, timeout, true, 127, false);
  intake.move_velocity(-300);
  pros::delay(WING_ACTIVATE_DELAY);
  intake.move_velocity(600);
  chassis.moveToPoint(10, -24, timeout);

  // out take triball
  chassis.turnToPoint(60, 0, timeout, true, 127, false);
  intake.move_velocity(-600);
  pros::delay(WING_ACTIVATE_DELAY);

  // go to next triball
  intake.move_velocity(600);
  chassis.moveToPoint(-0, -7, timeout);
  chassis.turnToPoint(60, 0, timeout, true, 127, true);
  intake.move_velocity(-600);
  pros::delay(WING_ACTIVATE_DELAY);

  // wing all in
  chassis.turnToPoint(-60, -6, timeout, true, 127, false);
  wings.set_value(true);
  chassis.moveToPoint(48, -6, timeout, {.forwards = false});
}
ASSET(returnback_txt);
void doing_6_ball() {
  // set pose
  chassis.setPose(14, -60, -90);
  chassis.moveToPoint(4, -60, 1400);
  chassis.waitUntil(5);
  intake.move_velocity(500);
  pros::delay(500);

  // do 2 back balls
  chassis.follow(returnback_txt, 15, 2300, false);
  chassis.waitUntil(25);
  // intake.brake();
  chassis.waitUntil(40.5);
  // wings.set_value(true);
  chassis.waitUntilDone();

  // push
  // chassis.moveToPoint(60, 24 + 7, 400, false, 127, false);
  wings.set_value(false);
  chassis.turnToHeading(200, 300, true, 60);
  chassis.waitUntilDone();
  chassis.tank(60, 60);
  pros::delay(300);
  chassis.tank(-127, -127);
  pros::delay(500);
  chassis.tank(0, 0);

  // outake internal triball
  wings.set_value(false);
  chassis.turnToHeading(0, 1200);
  chassis.waitUntilDone();
  chassis.tank(100, 100);
  pros::delay(200);
  intake.move_velocity(-600);
  pros::delay(300);
  chassis.tank(0, 0);

  // go to lower and outake
  chassis.moveToPoint(48, -48, 1800, {.forwards = false, .maxSpeed = 80});
  /*chassis.waitUntilDone();
  pros::delay(1000);*/
  chassis.turnToHeading(-90+26.5650512,1800);
  //chassis.waitUntilDone();
  //pros::delay(10000);
  //chassis.moveToPoint(6, -24, 1500);
  //intake.move_velocity(600);

  /*/ go to top
  intake.move_velocity(400);
  chassis.moveToPoint(6,0, 1400,{.maxSpeed = 90});
  chassis.waitUntilDone();
  pros::delay(400);

  // wing push
  chassis.turnToHeading(-90, 800);
  wings.set_value(true);
  chassis.moveToPoint(48, 0, 2000, {.forwards = false, .minSpeed = 110});

  // shoot top
  chassis.turnToHeading(90, 1200);
  chassis.waitUntilDone();
  intake.move_velocity(-300);
  pros::delay(300);

  // go to third
  chassis.moveToPoint(8, 0, 800);
  intake.move_velocity(300);
  chassis.waitUntilDone();
  pros::delay(600);

  // wing push
  chassis.turnToPoint(-60, 0, 800);
  chassis.waitUntilDone();
  wings.set_value(true);
  chassis.moveToPoint(41, 0, 1000, {.forwards = false});

  // put third in goal
  chassis.moveToPoint(40, 0, 1200);
  chassis.turnToPoint(60, 0, 800);
  chassis.waitUntilDone();
  intake.move(-127);
  pros::delay(400);
  chassis.moveToPoint(44, 0, 800);
  chassis.waitUntilDone();
  intake.move(-60);//*/
}

/// @brief do skills
void skills() {
  // start pos
  chassis.setPose(44, -56, -52);

  // shoot cata
  cata.move_velocity(100);
  pros::delay(35000);
  cata.brake();

  // clean field
  chassis.moveToPoint(40, 4, timeout);
  chassis.turnToPoint(60, 0, timeout, true, 127, false);
  wings.set_value(true);
  chassis.tank(-127, -127, 0);
  pros::delay(250);

  // go over
  wings.set_value(false);
  chassis.turnToPoint(-60, 0, timeout, true, 127, false);
  chassis.tank(-127, -127, 0);
  pros::delay(320);
  chassis.tank(127, 127, 0);
  pros::delay(3000);

  // turn around
  chassis.turnToPoint(chassis.getPose().x, 600, timeout);
  chassis.tank(-127, -127, 0);
  pros::delay(250);
  chassis.turnToPoint(60, chassis.getPose().y, timeout, true, 127, false);

  // wing push
  wings.set_value(true);
  pros::delay(150);
  chassis.tank(-127, -127, 0);
  pros::delay(2500);
  chassis.cancelAllMotions();
}
ASSET(skills_push_left_txt)
ASSET(skills_go_to_pp_txt);
ASSET(skills_go_to_pp_1_txt)
ASSET(skills_go_to_pp_2_txt)
ASSET(skills_push_right_after_main_push_txt)
void skills2() {
  const int triballs = 48;
  const int speed = 90;
  chassis.setPose(12, -60, -90);

  // move to matchload
  chassis.follow(skills_push_left_txt, 15, 3500, false);
  // chassis.turnToPoint(-60, 0, 1200, SWING_SPEED);
  // chassis.moveToPoint(49, -50, 1000, false, 40);
  chassis.turnToPoint(-60, 0, 1200, true, 90);
  chassis.waitUntilDone();
  chassis.tank(-30, -30);
  pros::delay(WING_ACTIVATE_DELAY);
  chassis.tank(0, 0);
  chassis.turnToPoint(-60, 0, 1200);
  chassis.waitUntilDone();

  // shoot triballs
  cata.move_velocity(speed);
  pros::delay(32e3);
  cata.move(0);
  chassis.setPose(47.5, -48 - 24 / 4.7, -45 - 10);

  // go to push
  chassis.follow(skills_go_to_pp_txt, 15, 8000);
  chassis.turnToPoint(-60, 0, 1200, false, SWING_SPEED);
  chassis.waitUntilDone();

  // first pushes
  int left_pushes = 2;
  for (int i = 0; i < left_pushes; i++) {
    wings.set_value(true);
    pros::delay(WING_ACTIVATE_DELAY);
    chassis.moveToPose(-48 + 8.25 - 10 - 4 * i, -3, 90, 1500,
                       {.forwards = false, .minSpeed = 127});
    chassis.waitUntilDone();
    wings.set_value(false);
    pros::delay(WING_ACTIVATE_DELAY);
    chassis.moveToPoint(-20, -2, 1200);
    chassis.waitUntilDone();
  }

  // set up for 2nd pushes
  chassis.turnToPoint(-7, 12, 1200);
  chassis.moveToPoint(-7, 12, 1200);
  chassis.turnToPoint(-60, 12, 1200, false);
  chassis.waitUntilDone();

  // 2nd pushes
  int right_pushes = 2;
  for (int i = 0; i < right_pushes; i++) {
    wings.set_value(true);
    pros::delay(WING_ACTIVATE_DELAY);
    chassis.moveToPose(-48 + 8.25 - 11 - 3 * i, 10 + 3, 90 + 10, 1500,
                       {.forwards = false, .minSpeed = 127});
    chassis.waitUntilDone();
    wings.set_value(false);
    pros::delay(WING_ACTIVATE_DELAY);
    chassis.moveToPoint(-7, 20 + 3, 1200);
    chassis.waitUntilDone();
  }

  /*/ right push
  chassis.follow(skills_push_right_after_main_push_txt, 15, 6000, false);
  chassis.waitUntil(42);
  wings.set_value(true);

  // 2nd push
  chassis.tank(30, 30);
  pros::delay(300);
  chassis.tank(-100, -100);
  pros::delay(500);//*/

  /*/ put triballs
  chassis.moveToPoint(60, -36, 1000, false, DRIVE_SPEED);
  chassis.turnToPoint(60, 0, 600, false, SWING_SPEED);
  // wings.set_value(true);
  chassis.waitUntilDone();
  chassis.tank(-127, -127);
  pros::delay(WING_ACTIVATE_DELAY);
  chassis.tank(60, 60);
  pros::delay(300);
  chassis.tank(-127, -127);
  pros::delay(WING_ACTIVATE_DELAY);
  chassis.tank(0, 0); //*/

  /*/ matchload
  chassis.moveToPoint(48 + 4, -48 - 1, 1500, true, DRIVE_SPEED);
  chassis.turnToPoint(-48, 0, 1200, true, SWING_SPEED);
  chassis.waitUntilDone();
  chassis.tank(-30, -30);
  pros::delay(300);
  chassis.tank(0, 0);
  chassis.turnToPoint(-60, 0, 300);
  // wings.set_value(true);
  cata.move_velocity(60);
  pros::delay(35000);
  cata.brake(); //*/

  /*/ push left side
  wings.set_value(false);
  chassis.follow(skills_push_left_txt, 15, 5000, false);//*/
}

/// @brief move the robot a number of inches forward
/// @param lateral_movement how many inches forward you desire the robot to move
void lateral_pid_tuning(int lateral_movement) {
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, lateral_movement, 1e4);
}

/// @brief a function for turning angular_movement degrees
/// @param angular_movement how many degrees you want to turn clockwise
void angular_pid_tuning(int angular_movement) {
  chassis.setPose(0, 0, 0);
  chassis.turnToHeading(angular_movement, 10e4);
}

/// @brief do a movement test
void combination_tuning() {
  int start = pros::millis();
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 48, 1e4);
  chassis.turnToPoint(48, 48, 1e4);
  chassis.moveToPoint(48, 48, 1e4);
  int end = pros::millis();
  pros::lcd::print(5, "Time: %f", (end - start) / 1000);
}