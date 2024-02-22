#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/timer.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"

// right motors
pros::Motor rF(11, pros::E_MOTOR_GEARSET_18, false);
pros::Motor rM(12, pros::E_MOTOR_GEARSET_18, false);
pros::Motor rT(13, pros::E_MOTOR_GEARSET_18, false);

// left motors
pros::Motor lF(14, pros::E_MOTOR_GEARSET_18, true);
pros::Motor lM(15, pros::E_MOTOR_GEARSET_18, true);
pros::Motor lT(16, pros::E_MOTOR_GEARSET_18, true);

// intake
pros::Motor intake(9, pros::E_MOTOR_GEARSET_06, true);

// blocker

pros::ADIDigitalOut hang(3, false);

// wings
bool wings_engaged = false;
pros::ADIDigitalOut wings(2, wings_engaged);

// cata: manual gear adjustment required
bool shooting_cata = false;

pros::Motor cata(8, pros::E_MOTOR_GEARSET_36, false);

// lemlib::PID pid(3, 0.1, 2);
//  make drivetrain
pros::MotorGroup left_motors({lT, lM, lF});
pros::MotorGroup right_motors({rT, rM, rF});

lemlib::Drivetrain drivetrain{
    &left_motors, &right_motors, 10.7, lemlib::Omniwheel::NEW_325, 1000 / 3, 2};

// sensors (only inertial right now)
pros::Imu inertial_sensor(4);

lemlib::OdomSensors sensors{nullptr,
 nullptr, nullptr, nullptr,
                            &inertial_sensor};

// PIDs

// forward/backward PID
lemlib::ControllerSettings lateralController{
    20,  // proportional gain (kP)
    0.2, // integral gain (kI)
    20,  // derivative gain (kD)
    3,   // anti windup
    0.5, // small error range, in inches
    200, // small error range timeout, in milliseconds
    1,   // large error range, in inches
    500, // large error range timeout, in milliseconds
    3    // maximum acceleration (slew)

};

// turning PID
lemlib::ControllerSettings angularController{
    3.3, // proportional gain (kP)
    0.1, // integral gain (kI)
    25,  // derivative gain (kD)
    3,   // anti windup
    1,   // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3,   // large error range, in degrees
    300, // large error range timeout, in milliseconds
    3    // maximum acceleration (slew)
};

// make chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController,
                        sensors);
pros::Controller master(pros::E_CONTROLLER_MASTER);

/*
-101: lateral pid tuning
-102: angular pid tuning
0: left side
1: right side
2: auto skills
*/
float auton_selector;
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}
lemlib::Timer timer(5000);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  chassis.calibrate();
  pros::lcd::set_text(7, "Chassis Calibrated :)");
  intake.set_brake_mode(MOTOR_BRAKE_BRAKE);
  cata.set_brake_mode(MOTOR_BRAKE_HOLD);

  pros::Task screenTask([&]() {
    lemlib::Pose pose(0, 0, 0);
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

      // log position telemetry
      lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
      printf("X: %f", chassis.getPose().x);
      printf("Y: %f", chassis.getPose().y);
      printf("Theta: %f", chassis.getPose().theta);
      // delay to save resources
      pros::delay(50);
    }
  });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {

  /*
  -103: combination tuning
  -101: lateral pid tuning
  -102: angular pid tuning
  0: left side
  1: right side
  2: auto skills
  */
  auton_selector = 0;

  left_motors.set_brake_modes(MOTOR_BRAKE_HOLD);
  right_motors.set_brake_modes(MOTOR_BRAKE_HOLD);

  if (auton_selector == 0) {
    defensive_side_safe();
  } else if (auton_selector == 0.5) {
    defensive_side_mess_up();
  } else if (auton_selector == 1) {
    doing_6_ball();
  } else if (auton_selector == 2) {
    skills2();
  } else if (auton_selector == -101) {
    lateral_pid_tuning(36); /* 10 inches */
  } else if (auton_selector == -102) {
    angular_pid_tuning(90);
  } else if (auton_selector == -103) {
    combination_tuning();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  left_motors.set_brake_modes(MOTOR_BRAKE_BRAKE);
  right_motors.set_brake_modes(MOTOR_BRAKE_BRAKE);
  int driver_slew = 255;
  int prev_right = 0;
  int prev_left = 0;
  while (true) {
    pros::lcd::print(5, "%d %d %d",
                     (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                     (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                     (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
    int left = master.get_analog(ANALOG_LEFT_Y);
    int right = master.get_analog(ANALOG_RIGHT_Y);

    chassis.tank(left, right, 3.5);

    prev_left = left;
    prev_right = right;

    if (master.get_digital(DIGITAL_R2)) {
      intake.move_velocity(600);
    } else if (master.get_digital(DIGITAL_R1)) {
      intake.move_velocity(-600);
    } else {
      intake.brake();
    }

    if (master.get_digital_new_press(DIGITAL_LEFT)) {
      wings_engaged = !wings_engaged;
      wings.set_value(wings_engaged);
    }
    if (master.get_digital_new_press(DIGITAL_RIGHT)) {
      hang.set_value(true);
    }

    if (master.get_digital_new_press(DIGITAL_L1)) {
      shooting_cata = !shooting_cata;
    }
    if (shooting_cata) {
      cata.move_velocity(100);
    } else {
      cata.brake();
    }
    pros::delay(20);
  }
}
