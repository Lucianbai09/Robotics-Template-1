#pragma once
#include "OVR/drive.h"

class Drive;

extern Drive chassis;
extern object_color alliance;

void default_constants();
void odom_constants();

void drive_test();
void turn_test();
void swing_test();
void full_test();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();

void example_auton();
void example_skills();
void async_example();
