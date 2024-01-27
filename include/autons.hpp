#pragma once

#include "lemlib/chassis/chassis.hpp"

extern lemlib::Chassis chassis;

extern pros::ADIDigitalOut wings;
extern pros::Motor intake;
extern pros::Motor cata;
extern pros::ADIDigitalOut blocker;

void defensive_side();
void defensive_side_safe();
void defensive_side_mess_up();
void offensive_side();
void doing_6_ball();
void skills();
void skills2();
void lateral_pid_tuning(int);
void angular_pid_tuning(int);
void combination_tuning();