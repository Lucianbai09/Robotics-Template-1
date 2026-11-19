#include "vex.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the README. This runs before every auton.
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);

  chassis.drive_min_voltage = 0;
  chassis.boomerang_lead = .5;
  chassis.boomerang_setback = 2;

  // Max volts the drive can speed up by every 10ms. 0.5 goes from 0 to 12 volts
  // in about a quarter second, which keeps a 55W drive from slipping. 0 turns it off.
  chassis.drive_slew = 0.5;

  // Ends a motion once the robot has been stuck this many ms, like when it's
  // pushed up against a Goal or Loader. 0 turns it off.
  chassis.stall_time = 0;
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
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

void drive_test(){
  chassis.drive_distance(6);
  chassis.drive_distance(12);
  chassis.drive_distance(18);
  chassis.drive_distance(-36);
}

/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */

void turn_test(){
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);
}

/**
 * Should swing in a fun S shape.
 */

void swing_test(){
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void full_test(){
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

void odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
}

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */

void tank_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24,24);
  chassis.drive_to_point(0,0);
  chassis.turn_to_angle(0);
}

/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */

void holonomic_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_pose(0, 18, 90);
  chassis.holonomic_drive_to_pose(18, 0, 180);
  chassis.holonomic_drive_to_pose(0, 18, 270);
  chassis.holonomic_drive_to_pose(0, 0, 0);
}

/**
 * EXAMPLE helper. Flips the Pin in the claw if the half facing the color
 * sensor isn't your alliance color. Which half you want up depends on your
 * strategy, since yellow halves only score for whoever owns the Toggle.
 */

void orient_pin(){
  object_color seen = get_color(ColorSensor);
  if(seen != COLOR_NONE && seen != alliance){
    Wrist.move_to(Wrist.target == WRIST_UP ? WRIST_FLIPPED : WRIST_UP, 700);
  }
}

/**
 * EXAMPLE match auton. The moves are relative to where the robot starts,
 * so the same code runs from either starting tile on your side: the
 * selector in main.cpp runs it again with mirror on for the other tile.
 * The distances are made up, so replace them with your own route.
 * For the AWP, by the end of auton you need 6 of your Pin halves Scored
 * on your side of the Autonomous Line, in 2+ Goals with 2+ halves each,
 * and neither robot touching the field perimeter. Events that qualify
 * for Worlds need 7 halves in 3 Goals.
 */

void example_auton(){
  chassis.set_coordinates(0, 0, 0);
  Lift.set_position(LIFT_DOWN);
  Wrist.set_position(WRIST_UP);
  Claw.set(true); // Grip the preload. On this example robot true closes the claw.

  // Raise the lift while driving to the first Goal.
  Lift.move_to(LIFT_ALLIANCE_GOAL);
  chassis.drive_distance(24);
  chassis.turn_to_angle(90);
  Lift.move_to(LIFT_ALLIANCE_GOAL, 1000); // Make sure the lift is up before scoring.

  // Push up to the Goal, but stop as soon as the robot is stuck against it.
  chassis.stall_time = 200;
  chassis.drive_distance(8);
  chassis.stall_time = 0;
  Claw.set(false);
  wait(200, msec);

  // Back off, lower the lift on the way, and grab the next Pin.
  chassis.drive_distance(-8);
  Lift.move_to(LIFT_DOWN);
  chassis.turn_to_angle(180);
  chassis.drive_distance(20);
  Claw.set(true);
  wait(200, msec);
  orient_pin();

  // Score it on the short Goal.
  Lift.move_to(LIFT_SHORT_GOAL);
  chassis.turn_to_angle(225);
  chassis.drive_distance(18);
  Claw.set(false);
  wait(200, msec);

  // Back away so neither robot ends touching the perimeter.
  chassis.drive_distance(-6);
}

/**
 * EXAMPLE skills start. Skills uses field coordinates, so the distance
 * sensors can reset odom off the walls during the minute. (0, 0) is the
 * center of the field and the walls are at +-70.25 inches. In skills,
 * Match Loads only go through the red Loaders.
 *
 * Override field map in inches, converted from the VEX GPS field map
 * (red alliance station on the -X wall). Check them on your field:
 * Tall Goal (0, 0). Short Goals (-23.6, 47.2), (-47.2, 23.6), (47.2, -23.6), (23.6, -47.2).
 * Red Goals (-47.2, -23.6), (-23.6, -47.2). Blue Goals (23.6, 47.2), (47.2, 23.6).
 * Toggles (0, 70.1), (70.1, 0), (0, -70.1), (-70.1, 0). Red Loaders (-68.5, 58.7), (-68.5, -58.7).
 */

void example_skills(){
  odom_constants();
  chassis.set_coordinates(-58, -40, 90); // Measure where your robot really starts.
  Lift.set_position(LIFT_DOWN);
  Wrist.set_position(WRIST_UP);
  Claw.set(true);

  // Line up below the red Goal at (-47.2, -23.6), facing it, with the lift going up on the way.
  Lift.move_to(LIFT_ALLIANCE_GOAL);
  chassis.drive_to_pose(-47.2, -36, 0);
  Claw.set(false);
  wait(200, msec);

  // Square to the walls, then let the distance sensors fix any drift.
  // reset_from_wall(sensor, inches right of center, inches in front of center, way it faces, max jump)
  chassis.turn_to_angle(0);
  chassis.reset_from_wall(LeftDistance, -7, 0, 270, 6);
  chassis.reset_from_wall(BackDistance, 0, -7, 180, 6);

  // Head to the bottom red Loader for a Match Load.
  Lift.move_to(LIFT_DOWN);
  chassis.drive_to_pose(-56, -58.7, 270);
  chassis.stall_time = 200;
  chassis.drive_distance(6);
  chassis.stall_time = 0;
  Claw.set(true);
  wait(200, msec);
  chassis.drive_distance(-6);
}

/**
 * Shows background motions. The drive runs in the background while
 * the code below it times the intake and lift to the drive.
 */

void async_example(){
  chassis.set_coordinates(0, 0, 0);

  // Drive in the background, and start the intake 12 inches in.
  chassis.run_async([](){ chassis.drive_distance(36); });
  chassis.wait_until_traveled(12);
  Intake.spin(fwd, 12, volt);

  // Stop the drive early once the color sensor sees a Pin.
  waitUntil(!chassis.is_moving() || get_color(ColorSensor) != COLOR_NONE);
  chassis.cancel_motion();
  chassis.wait_until_done();
  Intake.stop();

  // Turn, and raise the lift halfway through the turn.
  chassis.run_async([](){ chassis.turn_to_angle(90); });
  chassis.wait_until_turned(45);
  Lift.move_to(LIFT_SHORT_GOAL);

  // Motions without run_async() wait for the background ones, so this runs after the turn.
  chassis.drive_distance(12);
}
