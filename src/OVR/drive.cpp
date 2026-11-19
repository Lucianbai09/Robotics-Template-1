#include "vex.h"
#include <cmath>

/**
 * Drive constructor for the chassis.
 * Even though there's only one constructor, there can be
 * huge differences in implementation depending on the drive style
 * selected.
 * 
 * @param drive_setup The style of drive, such as TANK_TWO_ROTATION.
 * @param DriveL Left motor group.
 * @param DriveR Right motor group.
 * @param gyro_port IMU port.
 * @param wheel_diameter Wheel diameter in inches.
 * @param wheel_ratio External drive gear ratio.
 * @param gyro_scale Scale factor in degrees.
 * @param DriveLF_port Left front port for holonomic drives.
 * @param DriveRF_port Right front port for holonomic drives.
 * @param DriveLB_port Left back port for holonomic drives.
 * @param DriveRB_port Right back port for holonomic drives.
 * @param ForwardTracker_port Port for the forward tracker.
 * @param ForwardTracker_diameter Diameter in inches.
 * @param ForwardTracker_center_distance Horizontal distance in inches.
 * @param SidewaysTracker_port Port for the sideways tracker.
 * @param SidewaysTracker_diameter Diameter in inches.
 * @param SidewaysTracker_center_distance Vertical distance in inches.
 */

Drive::Drive(enum::drive_setup drive_setup, motor_group DriveL, motor_group DriveR, 
int gyro_port, float wheel_diameter, float wheel_ratio, float gyro_scale, 
int DriveLF_port, int DriveRF_port, int DriveLB_port, int DriveRB_port, 
int ForwardTracker_port, float ForwardTracker_diameter, float ForwardTracker_center_distance, 
int SidewaysTracker_port, float SidewaysTracker_diameter, float SidewaysTracker_center_distance) :
  wheel_diameter(wheel_diameter),
  wheel_ratio(wheel_ratio),
  gyro_scale(gyro_scale),
  drive_in_to_deg_ratio(wheel_ratio/360.0*M_PI*wheel_diameter),
  ForwardTracker_center_distance(ForwardTracker_center_distance),
  ForwardTracker_diameter(ForwardTracker_diameter),
  ForwardTracker_in_to_deg_ratio(M_PI*ForwardTracker_diameter/360.0),
  SidewaysTracker_center_distance(SidewaysTracker_center_distance),
  SidewaysTracker_diameter(SidewaysTracker_diameter),
  SidewaysTracker_in_to_deg_ratio(M_PI*SidewaysTracker_diameter/360.0),
  drive_setup(drive_setup),
  DriveL(DriveL),
  DriveR(DriveR),
  Gyro(inertial(gyro_port)),
  DriveLF(abs(DriveLF_port), is_reversed(DriveLF_port)),
  DriveRF(abs(DriveRF_port), is_reversed(DriveRF_port)),
  DriveLB(abs(DriveLB_port), is_reversed(DriveLB_port)),
  DriveRB(abs(DriveRB_port), is_reversed(DriveRB_port)),
  R_ForwardTracker(ForwardTracker_port),
  R_SidewaysTracker(SidewaysTracker_port),
  E_ForwardTracker(ThreeWire.Port[to_port(ForwardTracker_port)]),
  E_SidewaysTracker(ThreeWire.Port[to_port(SidewaysTracker_port)])
{
    if (drive_setup == TANK_ONE_FORWARD_ENCODER || drive_setup == TANK_ONE_FORWARD_ROTATION || drive_setup == ZERO_TRACKER_ODOM){
      odom.set_physical_distances(ForwardTracker_center_distance, 0);
    } 
    if (drive_setup == TANK_ONE_SIDEWAYS_ENCODER || drive_setup == TANK_ONE_SIDEWAYS_ROTATION || 
    drive_setup == TANK_TWO_ENCODER || drive_setup == TANK_TWO_ROTATION ||
    drive_setup == HOLONOMIC_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ROTATION){
      odom.set_physical_distances(ForwardTracker_center_distance, SidewaysTracker_center_distance);
    }
}

/**
 * Drives each side of the chassis at the specified voltage.
 * 
 * @param leftVoltage Voltage out of 12.
 * @param rightVoltage Voltage out of 12.
 */

void Drive::drive_with_voltage(float leftVoltage, float rightVoltage){
  DriveL.spin(fwd, leftVoltage, volt);
  DriveR.spin(fwd, rightVoltage,volt);
}

/**
 * Resets default turn constants.
 * Turning includes turn_to_angle() and turn_to_point().
 * 
 * @param turn_max_voltage Max voltage out of 12.
 * @param turn_kp Proportional constant.
 * @param turn_ki Integral constant.
 * @param turn_kd Derivative constant.
 * @param turn_starti Minimum angle in degrees for integral to begin.
 */

void Drive::set_turn_constants(float turn_max_voltage, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  this->turn_max_voltage = turn_max_voltage;
  this->turn_kp = turn_kp;
  this->turn_ki = turn_ki;
  this->turn_kd = turn_kd;
  this->turn_starti = turn_starti;
} 

/**
 * Resets default drive constants.
 * Driving includes drive_distance(), drive_to_point(), and
 * holonomic_drive_to_point().
 * 
 * @param drive_max_voltage Max voltage out of 12.
 * @param drive_kp Proportional constant.
 * @param drive_ki Integral constant.
 * @param drive_kd Derivative constant.
 * @param drive_starti Minimum distance in inches for integral to begin
 */

void Drive::set_drive_constants(float drive_max_voltage, float drive_kp, float drive_ki, float drive_kd, float drive_starti){
  this->drive_max_voltage = drive_max_voltage;
  this->drive_kp = drive_kp;
  this->drive_ki = drive_ki;
  this->drive_kd = drive_kd;
  this->drive_starti = drive_starti;
} 

/**
 * Resets default heading constants.
 * Heading control keeps the robot facing the right direction
 * and is part of drive_distance(), drive_to_point(), and
 * holonomic_drive_to_point.
 * 
 * @param heading_max_voltage Max voltage out of 12.
 * @param heading_kp Proportional constant.
 * @param heading_ki Integral constant.
 * @param heading_kd Derivative constant.
 * @param heading_starti Minimum angle in degrees for integral to begin.
 */

void Drive::set_heading_constants(float heading_max_voltage, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  this->heading_max_voltage = heading_max_voltage;
  this->heading_kp = heading_kp;
  this->heading_ki = heading_ki;
  this->heading_kd = heading_kd;
  this->heading_starti = heading_starti;
}

/**
 * Resets default swing constants.
 * Swing control holds one side of the drive still and turns with the other.
 * Only left_swing_to_angle() and right_swing_to_angle() use these constants.
 * 
 * @param swing_max_voltage Max voltage out of 12.
 * @param swing_kp Proportional constant.
 * @param swing_ki Integral constant.
 * @param swing_kd Derivative constant.
 * @param swing_starti Minimum angle in degrees for integral to begin.
 */

void Drive::set_swing_constants(float swing_max_voltage, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  this->swing_max_voltage = swing_max_voltage;
  this->swing_kp = swing_kp;
  this->swing_ki = swing_ki;
  this->swing_kd = swing_kd;
  this->swing_starti = swing_starti;
} 

/**
 * Resets default turn exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param turn_settle_error Error to be considered settled in degrees.
 * @param turn_settle_time Time to be considered settled in milliseconds.
 * @param turn_timeout Time before quitting and move on in milliseconds.
 */

void Drive::set_turn_exit_conditions(float turn_settle_error, float turn_settle_time, float turn_timeout){
  this->turn_settle_error = turn_settle_error;
  this->turn_settle_time = turn_settle_time;
  this->turn_timeout = turn_timeout;
}

/**
 * Resets default drive exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param drive_settle_error Error to be considered settled in inches.
 * @param drive_settle_time Time to be considered settled in milliseconds.
 * @param drive_timeout Time before quitting and move on in milliseconds.
 */

void Drive::set_drive_exit_conditions(float drive_settle_error, float drive_settle_time, float drive_timeout){
  this->drive_settle_error = drive_settle_error;
  this->drive_settle_time = drive_settle_time;
  this->drive_timeout = drive_timeout;
}

/**
 * Resets default swing exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param swing_settle_error Error to be considered settled in degrees.
 * @param swing_settle_time Time to be considered settled in milliseconds.
 * @param swing_timeout Time before quitting and move on in milliseconds.
 */

void Drive::set_swing_exit_conditions(float swing_settle_error, float swing_settle_time, float swing_timeout){
  this->swing_settle_error = swing_settle_error;
  this->swing_settle_time = swing_settle_time;
  this->swing_timeout = swing_timeout;
}

/**
 * Gives the drive's heading with Gyro correction, ignoring mirror.
 * All the motion code works in this real heading.
 *
 * @return Gyro scale-corrected heading in the range [0, 360).
 */

float Drive::real_heading(){
  return( reduce_0_to_360( Gyro.rotation()*360.0/gyro_scale ) );
}

/**
 * Gives the drive's absolute heading with Gyro correction.
 * When mirror is on, this is the mirrored heading your auton sees.
 *
 * @return Gyro scale-corrected heading in the range [0, 360).
 */

float Drive::get_absolute_heading(){
  return( reduce_0_to_360( mirror_angle(real_heading()) ) );
}

/**
 * Mirroring flips the field left to right, so an auton written for
 * one side runs on the other. X and angles get negated, and left and
 * right swings trade places. Going from your auton to the real robot
 * and back is the same flip.
 *
 * @param X_position An X position in inches.
 * @return The mirrored X if mirror is on.
 */

float Drive::mirror_x(float X_position){
  if(mirror){ return(-X_position); }
  return(X_position);
}

/**
 * @param angle An angle in degrees.
 * @return The mirrored angle if mirror is on.
 */

float Drive::mirror_angle(float angle){
  if(mirror){ return(-angle); }
  return(angle);
}

/**
 * Gets the motor group's position and converts to inches.
 * Motor groups report their first motor, so put a motor that matches
 * wheel_ratio first in each group if you mix 11W and 5.5W motors.
 *
 * @return Left position in inches.
 */

float Drive::get_left_position_in(){
  return( DriveL.position(deg)*drive_in_to_deg_ratio );
}

/**
 * Gets the motor group's position and converts to inches.
 *
 * @return Right position in inches.
 */

float Drive::get_right_position_in(){
  return( DriveR.position(deg)*drive_in_to_deg_ratio );
}

/**
 * Distance driven forward, used by drive_distance().
 * Uses the forward tracking wheel if there is one, since it doesn't
 * slip like the drive wheels do. Otherwise it averages both sides.
 *
 * @return Forward position in inches.
 */

float Drive::get_forward_position_in(){
  if (drive_setup == TANK_ONE_FORWARD_ENCODER || drive_setup == TANK_ONE_FORWARD_ROTATION || drive_setup == TANK_TWO_ENCODER || drive_setup == TANK_TWO_ROTATION){
    return( get_ForwardTracker_position() );
  }
  return( (get_left_position_in()+get_right_position_in())/2.0 );
}

/**
 * Average wheel speed, used by the stall exit. Counts turning too,
 * since both sides spin when the robot turns in place.
 *
 * @return Speed in inches per second.
 */

float Drive::get_speed(){
  if (drive_setup == HOLONOMIC_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ROTATION){
    return( (fabs(DriveLF.velocity(rpm))+fabs(DriveRF.velocity(rpm))+fabs(DriveLB.velocity(rpm))+fabs(DriveRB.velocity(rpm)))/4.0*6.0*drive_in_to_deg_ratio );
  }
  return( (fabs(DriveL.velocity(rpm))+fabs(DriveR.velocity(rpm)))/2.0*6.0*drive_in_to_deg_ratio );
}

/**
 * Stops both sides of the drive with the desired mode.
 *
 * @param mode hold, brake, or stop
 */

void Drive::drive_stop(vex::brakeType mode){
  DriveL.stop(mode);
  DriveR.stop(mode);
  if (drive_setup == HOLONOMIC_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ROTATION){
    DriveLF.stop(mode);
    DriveRF.stop(mode);
    DriveLB.stop(mode);
    DriveRB.stop(mode);
  }
}

/**
 * Runs at the start of every motion. If a motion is already running
 * (like one started with run_async()), this waits for it to finish, so
 * motions always run one at a time in the order they were called.
 * Each motion gets an id, so stop_motions() can end it from anywhere.
 * 
 * @return The id of this motion.
 */

int Drive::start_motion(){
  while(active_motion != 0){ task::sleep(5); }
  check_odom();
  cancel_requested = false;
  traveled = 0;
  turned = 0;
  motion_time = 0;
  stalled_time = 0;
  start_rotation = Gyro.rotation()*360.0/gyro_scale;
  start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  last_X = odom.X_position;
  last_Y = odom.Y_position;
  motion_count++;
  active_motion = motion_count;
  return(active_motion);
}

/**
 * Runs at the end of every motion. Like JAR, the drive is left at its
 * last voltage so chained motions flow into each other, unless the
 * motion was cancelled, in which case the drive brakes.
 * 
 * @param motion The id from start_motion().
 */

void Drive::end_motion(int motion){
  if(active_motion != motion){ return; } // Already ended by stop_motions().
  if(cancel_requested){ drive_stop(brake); }
  active_motion = 0;
}

/**
 * Loop condition for every motion. Keeps track of how far the robot
 * has gone for the wait functions, and ends the motion early if it was
 * cancelled or the robot has been stuck for stall_time. The stall exit
 * ignores the first 300ms so the robot has time to get moving.
 * 
 * @param settled Whether the motion's PID has settled.
 * @param motion The id from start_motion().
 * @return Whether the motion should keep going.
 */

bool Drive::motion_active(bool settled, int motion){
  if(active_motion != motion){ return(false); }
  if(odom_running){
    traveled += hypot(odom.X_position-last_X, odom.Y_position-last_Y);
    last_X = odom.X_position;
    last_Y = odom.Y_position;
  } else {
    traveled = fabs((get_left_position_in()+get_right_position_in())/2.0-start_average_position);
  }
  turned = fabs(Gyro.rotation()*360.0/gyro_scale-start_rotation);

  motion_time += 10;
  if(stall_time > 0 && motion_time > 300 && get_speed() < 1){
    stalled_time += 10;
  } else {
    stalled_time = 0;
  }
  if(stall_time > 0 && stalled_time >= stall_time){ return(false); }
  if(cancel_requested){ return(false); }
  return(!settled);
}

/**
 * Runs motions in the background so the rest of the auton keeps going,
 * like spinning the intake or raising the lift while driving. Pass a
 * function with no inputs, which can be a lambda:
 * chassis.run_async([](){ chassis.drive_to_point(24, 24); });
 * Motions called after this wait for the background ones to finish.
 * Only one background function runs at a time, so don't call
 * run_async() from inside a background function.
 *
 * @param motion Function that runs the motion(s). Start it with a motion.
 */

void Drive::run_async(void (*motion)()){
  wait_until_done();
  async_motion = motion;
  async_running = true;
  async_handle = task(async_task);
  // Wait for the motion to start, so motions called after this queue up behind it.
  while(async_running && active_motion == 0){ task::sleep(1); }
}

/**
 * Background task for run_async().
 */

int Drive::async_task(){
  chassis.async_motion();
  chassis.async_running = false;
  return(0);
}

/**
 * Waits until the current motion has driven a distance, or has ended.
 * Distance is counted from the start of the current motion.
 *
 * @param distance Distance in inches.
 */

void Drive::wait_until_traveled(float distance){
  task::sleep(10);
  while((active_motion != 0 || async_running) && traveled < distance){ task::sleep(10); }
}

/**
 * Waits until the current motion has turned a number of degrees, or has ended.
 *
 * @param degrees Degrees turned since the start of the current motion.
 */

void Drive::wait_until_turned(float degrees){
  task::sleep(10);
  while((active_motion != 0 || async_running) && turned < degrees){ task::sleep(10); }
}

/**
 * Waits until everything started with run_async() is done.
 */

void Drive::wait_until_done(){
  while(async_running){ task::sleep(10); }
}

/**
 * Ends the motion that is running now and brakes the drive. Use it to
 * stop a background motion early, like when the claw sensor sees a Pin.
 * Motions after it in the same run_async() function still run.
 */

void Drive::cancel_motion(){
  if(active_motion != 0){ cancel_requested = true; }
}

/**
 * Stops every motion, including background ones. main.cpp runs this at
 * the start of autonomous and driver control, because a motion that was
 * running when the last period ended got cut off and never finished.
 */

void Drive::stop_motions(){
  if(async_running){ async_handle.stop(); }
  async_running = false;
  active_motion = 0;
  cancel_requested = false;
}

/**
 * @return Whether a motion is running, in the background or not.
 */

bool Drive::is_moving(){
  return(active_motion != 0 || async_running);
}

/**
 * Turns the robot to a field-centric angle.
 * Optimizes direction, so it turns whichever way is closer to the
 * current heading of the robot.
 *
 * @param angle Desired angle in degrees.
 */

void Drive::turn_to_angle(float angle){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  angle = mirror_angle(angle);
  int motion = start_motion();
  PID turnPID(reduce_negative_180_to_180(angle - real_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while( motion_active(turnPID.is_settled(), motion) ){
    float error = reduce_negative_180_to_180(angle - real_heading());
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    task::sleep(10);
  }
  end_motion(motion);
}

/**
 * Drives the robot a given distance with a given heading.
 * Drive distance does not optimize for direction, so it won't try
 * to drive at the opposite heading from the one given to get there faster.
 * You can control the heading, but if you choose not to, it will drive with the
 * heading it's facing when the motion starts. It uses the forward tracker if
 * there is one, otherwise the average of the left and right motor groups.
 *
 * @param distance Desired distance in inches.
 * @param heading Desired heading in degrees.
 */

void Drive::drive_distance(float distance){
  drive_distance(distance, NAN, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  int motion = start_motion();
  // NAN means "keep the heading the robot has now", read after any earlier motion is done.
  if(std::isnan(heading)){ heading = get_absolute_heading(); }
  heading = mirror_angle(heading);
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - real_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_position = get_forward_position_in();
  float previous_drive_output = (DriveL.voltage(volt)+DriveR.voltage(volt))/2.0;
  while(motion_active(drivePID.is_settled(), motion)){
    float drive_error = distance+start_position-get_forward_position_in();
    float heading_error = reduce_negative_180_to_180(heading - real_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    drive_output = slew(drive_output, previous_drive_output, drive_slew);
    previous_drive_output = drive_output;
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
    task::sleep(10);
  }
  end_motion(motion);
}

/**
 * Turns to a given angle with only one side of the drivetrain.
 * Like turn_to_angle(), is optimized for turning the shorter
 * direction.
 *
 * @param angle Desired angle in degrees.
 */

void Drive::left_swing_to_angle(float angle){
  swing_to_angle(angle, true, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::left_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  swing_to_angle(angle, true, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::right_swing_to_angle(float angle){
  swing_to_angle(angle, false, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::right_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  swing_to_angle(angle, false, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

/**
 * Shared code for both swings. When mirror is on, the other side
 * of the drive does the swing.
 *
 * @param angle Desired angle in degrees.
 * @param left_side True to swing with the left side.
 */

void Drive::swing_to_angle(float angle, bool left_side, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  angle = mirror_angle(angle);
  if(mirror){ left_side = !left_side; }
  int motion = start_motion();
  PID swingPID(reduce_negative_180_to_180(angle - real_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(motion_active(swingPID.is_settled(), motion)){
    float error = reduce_negative_180_to_180(angle - real_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -swing_max_voltage, swing_max_voltage);
    if(left_side){
      DriveL.spin(fwd, output, volt);
      DriveR.stop(hold);
    } else {
      DriveR.spin(reverse, output, volt);
      DriveL.stop(hold);
    }
    task::sleep(10);
  }
  end_motion(motion);
}

/**
 * Depending on the drive style, gets the tracker's position.
 *
 * @return The tracker position.
 */

float Drive::get_ForwardTracker_position(){
  if (drive_setup==ZERO_TRACKER_ODOM || drive_setup == TANK_ONE_SIDEWAYS_ENCODER || drive_setup == TANK_ONE_SIDEWAYS_ROTATION){
    return(get_right_position_in());
  }
  if (drive_setup==TANK_ONE_FORWARD_ENCODER || drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER){
    return(E_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
  }else{
    return(R_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
  }
}

/**
 * Depending on the drive style, gets the tracker's position.
 *
 * @return The tracker position.
 */

float Drive::get_SidewaysTracker_position(){
  if (drive_setup==TANK_ONE_FORWARD_ENCODER || drive_setup == TANK_ONE_FORWARD_ROTATION || drive_setup == ZERO_TRACKER_ODOM){
    return(0);
  }else if (drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER || drive_setup == TANK_ONE_SIDEWAYS_ENCODER){
    return(E_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
  }else{
    return(R_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
  }
}

/**
 * Background task for updating the odometry.
 */

void Drive::position_track(){
  while(1){
    odom.update_position(get_ForwardTracker_position(), get_SidewaysTracker_position(), real_heading());
    odom_heartbeat.clear();
    task::sleep(5);
  }
}

/**
 * Starts the odom task if odom is in use and the task isn't running.
 * Competition control can stop tasks between autonomous and driver
 * control, so this restarts it when it hasn't run in a while.
 */

void Drive::check_odom(){
  if(odom_running && odom_heartbeat.time(msec) > 50){
    odom_heartbeat.clear();
    odom_task = task(position_track_task);
  }
}

/**
 * Resets the robot's heading.
 * For example, at the beginning of auton, if your robot starts at
 * 45 degrees, so set_heading(45) and the robot will know which way
 * it's facing.
 *
 * @param orientation_deg Desired heading in degrees.
 */

void Drive::set_heading(float orientation_deg){
  Gyro.setRotation(mirror_angle(orientation_deg)*gyro_scale/360.0, deg);
  odom.orientation_deg = real_heading();
}

/**
 * Resets the robot's coordinates and heading.
 * This is for odom-using robots to specify where the bot is at the beginning
 * of the match. The odom task starts the first time this is called.
 *
 * @param X_position Robot's x in inches.
 * @param Y_position Robot's y in inches.
 * @param orientation_deg Desired heading in degrees.
 */

void Drive::set_coordinates(float X_position, float Y_position, float orientation_deg){
  set_heading(orientation_deg);
  odom.set_position(mirror_x(X_position), Y_position, real_heading(), get_ForwardTracker_position(), get_SidewaysTracker_position());
  odom_running = (drive_setup != ZERO_TRACKER_NO_ODOM);
  check_odom();
}

/**
 * Gets the robot's x.
 *
 * @return The robot's x position in inches.
 */

float Drive::get_X_position(){
  return(mirror_x(odom.X_position));
}

/**
 * Gets the robot's y.
 *
 * @return The robot's y position in inches.
 */

float Drive::get_Y_position(){
  return(odom.Y_position);
}

/**
 * Resets X or Y using a distance sensor pointed at a field wall, so
 * odom stops drifting during a long skills run. Needs field coordinates
 * with (0, 0) at the center of the field, so the walls are at
 * +-field_half_size. The sensor must be within 15 degrees of square to
 * the wall and see the wall itself, not a Toggle, Loader, or robot.
 *
 * @param sensor The distance sensor.
 * @param sensor_x Inches the sensor sits right of the robot's center (left is negative).
 * @param sensor_y Inches the sensor sits in front of the robot's center (behind is negative).
 * @param sensor_angle Way the sensor faces on the robot: 0 front, 90 right, 180 back, 270 left.
 * @param max_jump Skips the reset if it would move the robot more than this many inches, 0 for no limit.
 * @return Whether the position was reset.
 */

bool Drive::reset_from_wall(vex::distance &sensor, float sensor_x, float sensor_y, float sensor_angle){
  return(reset_from_wall(sensor, sensor_x, sensor_y, sensor_angle, 0));
}

bool Drive::reset_from_wall(vex::distance &sensor, float sensor_x, float sensor_y, float sensor_angle, float max_jump){
  if(!sensor.isObjectDetected()){ return(false); }
  float reading = sensor.objectDistance(inches);
  if(reading <= 0 || reading > 78){ return(false); } // The sensor reads up to about 78 inches.

  float heading_rad = to_rad(real_heading());
  float beam_deg = reduce_0_to_360(real_heading()+sensor_angle);
  int wall = (int)round(beam_deg/90.0);
  if(fabs(beam_deg-wall*90) > 15){ return(false); }
  wall = wall%4; // 0 is the +Y wall, 1 is +X, 2 is -Y, 3 is -X.

  // Where the sensor is compared to the robot's center, and where the beam goes, in field directions.
  float offset_X = sensor_x*cos(heading_rad) + sensor_y*sin(heading_rad);
  float offset_Y = -sensor_x*sin(heading_rad) + sensor_y*cos(heading_rad);
  float beam_X = reading*sin(to_rad(beam_deg));
  float beam_Y = reading*cos(to_rad(beam_deg));

  if(wall == 0 || wall == 2){
    float wall_Y = (wall == 0) ? field_half_size : -field_half_size;
    float new_Y = wall_Y-beam_Y-offset_Y;
    if(max_jump > 0 && fabs(new_Y-odom.Y_position) > max_jump){ return(false); }
    odom.Y_position = new_Y;
  } else {
    float wall_X = (wall == 1) ? field_half_size : -field_half_size;
    float new_X = wall_X-beam_X-offset_X;
    if(max_jump > 0 && fabs(new_X-odom.X_position) > max_jump){ return(false); }
    odom.X_position = new_X;
  }
  return(true);
}

/**
 * Drives to a specified point on the field.
 * Uses the double-PID method, with one for driving and one for heading correction.
 * The drive error is the euclidean distance to the desired point, and the heading error
 * is the turn correction from the current heading to the desired point. Uses optimizations
 * like driving backwards whenever possible and scaling the drive output with the cosine
 * of the angle to the point.
 *
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 */

void Drive::drive_to_point(float X_position, float Y_position){
  drive_to_point(X_position, Y_position, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage){
  drive_to_point(X_position, Y_position, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_point(X_position, Y_position, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  X_position = mirror_x(X_position);
  int motion = start_motion();
  PID drivePID(hypot(X_position-odom.X_position,Y_position-odom.Y_position), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  float start_angle_deg = to_deg(atan2(X_position-odom.X_position,Y_position-odom.Y_position));
  PID headingPID(start_angle_deg-real_heading(), heading_kp, heading_ki, heading_kd, heading_starti);
  bool line_settled = false;
  bool prev_line_settled = is_line_settled(X_position, Y_position, start_angle_deg, odom.X_position, odom.Y_position);
  float previous_drive_output = (DriveL.voltage(volt)+DriveR.voltage(volt))/2.0;
  while(motion_active(drivePID.is_settled(), motion)){
    line_settled = is_line_settled(X_position, Y_position, start_angle_deg, odom.X_position, odom.Y_position);
    if(line_settled && !prev_line_settled){ break; }
    prev_line_settled = line_settled;

    float drive_error = hypot(X_position-odom.X_position,Y_position-odom.Y_position);
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-odom.X_position,Y_position-odom.Y_position))-real_heading());
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);

    if (drive_error<drive_settle_error) { heading_output = 0; }

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    drive_output = slew(drive_output, previous_drive_output, drive_slew);
    previous_drive_output = drive_output;
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

    drive_with_voltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
    task::sleep(10);
  }
  end_motion(motion);
}

/**
 * Drives to a specified point and orientation on the field.
 * Uses a boomerang controller. The carrot point is back from the target
 * by the same distance as the robot's distance to the target, times the lead. The
 * robot always tries to go to the carrot, which is constantly moving, and the
 * robot eventually gets into position. The heading correction is optimized to only
 * try to reach the correct angle when drive error is low. The robot always ends
 * front-first at the requested angle (JAR could end up backwards). To back into
 * a spot, use drive_to_point() or drive_distance() with a negative distance.
 * .5 is a reasonable value for the lead.
 * The setback parameter is used to glide into position more effectively. It is
 * the distance back from the target that the robot tries to drive to first.
 *
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 * @param angle Desired orientation in degrees.
 * @param lead Constant scale factor that determines how far away the carrot point is.
 * @param setback Distance in inches from target by which the carrot is always pushed back.
 * @param drive_min_voltage Minimum voltage on the drive, used for chaining movements.
 */

void Drive::drive_to_pose(float X_position, float Y_position, float angle){
  drive_to_pose(X_position, Y_position, angle, boomerang_lead, boomerang_setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage){
  drive_to_pose(X_position, Y_position, angle, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage){
  drive_to_pose(X_position, Y_position, angle, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_pose(X_position, Y_position, angle, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  X_position = mirror_x(X_position);
  angle = mirror_angle(angle);
  int motion = start_motion();
  float target_distance = hypot(X_position-odom.X_position,Y_position-odom.Y_position);
  PID drivePID(target_distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(to_deg(atan2(X_position-odom.X_position,Y_position-odom.Y_position))-real_heading(), heading_kp, heading_ki, heading_kd, heading_starti);
  bool line_settled = is_line_settled(X_position, Y_position, angle, odom.X_position, odom.Y_position);
  bool prev_line_settled = is_line_settled(X_position, Y_position, angle, odom.X_position, odom.Y_position);
  bool crossed_center_line = false;
  bool center_line_side = is_line_settled(X_position, Y_position, angle+90, odom.X_position, odom.Y_position);
  bool prev_center_line_side = center_line_side;
  float previous_drive_output = (DriveL.voltage(volt)+DriveR.voltage(volt))/2.0;
  while(motion_active(drivePID.is_settled(), motion)){
    line_settled = is_line_settled(X_position, Y_position, angle, odom.X_position, odom.Y_position);
    if(line_settled && !prev_line_settled){ break; }
    prev_line_settled = line_settled;

    center_line_side = is_line_settled(X_position, Y_position, angle+90, odom.X_position, odom.Y_position);
    if(center_line_side != prev_center_line_side){
      crossed_center_line = true;
    }

    target_distance = hypot(X_position-odom.X_position,Y_position-odom.Y_position);

    float carrot_X = X_position - sin(to_rad(angle)) * (lead * target_distance + setback);
    float carrot_Y = Y_position - cos(to_rad(angle)) * (lead * target_distance + setback);

    float drive_error = hypot(carrot_X-odom.X_position,carrot_Y-odom.Y_position);
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(carrot_X-odom.X_position,carrot_Y-odom.Y_position))-real_heading());

    if (drive_error<drive_settle_error || crossed_center_line || drive_error < setback) {
      heading_error = reduce_negative_180_to_180(angle-real_heading());
      drive_error = target_distance;
    }

    float drive_output = drivePID.compute(drive_error);

    // Unlike drive_to_point(), the heading error isn't reduced to [-90, 90), so
    // the robot always ends front-first at the requested angle instead of backwards.
    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    drive_output = slew(drive_output, previous_drive_output, drive_slew);
    previous_drive_output = drive_output;
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

    drive_with_voltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
    task::sleep(10);
  }
  end_motion(motion);
}

/**
 * Turns to a specified point on the field.
 * Functions similarly to turn_to_angle() except with a point. The
 * extra_angle_deg parameter turns the robot extra relative to the
 * desired target. For example, if you want the back of your robot
 * to point at (36, 42), you would run turn_to_point(36, 42, 180).
 *
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 * @param extra_angle_deg Angle turned past the desired heading in degrees.
 */

void Drive::turn_to_point(float X_position, float Y_position){
  turn_to_point(X_position, Y_position, 0, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  X_position = mirror_x(X_position);
  extra_angle_deg = mirror_angle(extra_angle_deg);
  int motion = start_motion();
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(X_position-odom.X_position,Y_position-odom.Y_position)) - real_heading() + extra_angle_deg), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(motion_active(turnPID.is_settled(), motion)){
    float error = reduce_negative_180_to_180(to_deg(atan2(X_position-odom.X_position,Y_position-odom.Y_position)) - real_heading() + extra_angle_deg);
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    task::sleep(10);
  }
  end_motion(motion);
}

/**
 * Drives and turns simultaneously to a desired pose.
 * Uses two PID loops, one drive and one heading to drive and turn
 * at the same time. Optimized to turn the quicker direction and only
 * exits once both PID loops have settled. It uses the heading constants
 * for heading but the turn exit conditions to settle.
 *
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 * @param angle Desired ending angle in degrees.
 */

void Drive::holonomic_drive_to_pose(float X_position, float Y_position){
  holonomic_drive_to_pose(X_position, Y_position, NAN, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_pose(float X_position, float Y_position, float angle){
  holonomic_drive_to_pose(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_pose(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage){
  holonomic_drive_to_pose(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_pose(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  holonomic_drive_to_pose(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_pose(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  X_position = mirror_x(X_position);
  int motion = start_motion();
  // NAN means "keep the heading the robot has now", read after any earlier motion is done.
  if(std::isnan(angle)){ angle = get_absolute_heading(); }
  angle = mirror_angle(angle);
  PID drivePID(hypot(X_position-odom.X_position,Y_position-odom.Y_position), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID turnPID(angle-real_heading(), heading_kp, heading_ki, heading_kd, heading_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while( motion_active(drivePID.is_settled() && turnPID.is_settled(), motion) ){
    float drive_error = hypot(X_position-odom.X_position,Y_position-odom.Y_position);
    float turn_error = reduce_negative_180_to_180(angle-real_heading());

    float drive_output = drivePID.compute(drive_error);
    float turn_output = turnPID.compute(turn_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    turn_output = clamp(turn_output, -heading_max_voltage, heading_max_voltage);

    float heading_error = atan2(Y_position-odom.Y_position, X_position-odom.X_position);

    DriveLF.spin(fwd, drive_output*cos(to_rad(real_heading()) + heading_error - M_PI/4) + turn_output, volt);
    DriveLB.spin(fwd, drive_output*cos(-to_rad(real_heading()) - heading_error + 3*M_PI/4) + turn_output, volt);
    DriveRB.spin(fwd, drive_output*cos(to_rad(real_heading()) + heading_error - M_PI/4) - turn_output, volt);
    DriveRF.spin(fwd, drive_output*cos(-to_rad(real_heading()) - heading_error + 3*M_PI/4) - turn_output, volt);
    task::sleep(10);
  }
  end_motion(motion);
}

/**
 * Applies the deadband and curve to a joystick reading.
 * A joystick_curve of 0 is linear. Higher values, up to 1, give finer
 * control at low speeds while keeping full speed at full stick.
 *
 * @param input Joystick reading in percent.
 * @return Curved reading in percent.
 */

float Drive::joystick(float input){
  input = deadband(input, joystick_deadband);
  return( joystick_curve*pow(input/100.0, 3)*100.0 + (1-joystick_curve)*input );
}

/**
 * Controls a chassis with left stick throttle and right stick turning.
 * Turning keeps priority at full throttle, so you can still steer.
 */

void Drive::control_arcade(){
  float throttle = to_volt(joystick(Controller.Axis3.position()));
  float turn = to_volt(joystick(Controller.Axis1.position()));
  DriveL.spin(fwd, left_voltage_scaling(throttle, turn), volt);
  DriveR.spin(fwd, right_voltage_scaling(throttle, turn), volt);
}

/**
 * Controls a chassis with left stick throttle and strafe, and right stick turning.
 */

void Drive::control_holonomic(){
  float throttle = joystick(Controller.Axis3.position());
  float turn = joystick(Controller.Axis1.position());
  float strafe = joystick(Controller.Axis4.position());
  DriveLF.spin(fwd, to_volt(throttle+turn+strafe), volt);
  DriveRF.spin(fwd, to_volt(throttle-turn-strafe), volt);
  DriveLB.spin(fwd, to_volt(throttle+turn-strafe), volt);
  DriveRB.spin(fwd, to_volt(throttle-turn+strafe), volt);
}

/**
 * Controls a chassis with left stick left drive and right stick right drive.
 */

void Drive::control_tank(){
  float leftthrottle = joystick(Controller.Axis3.position());
  float rightthrottle = joystick(Controller.Axis2.position());
  DriveL.spin(fwd, to_volt(leftthrottle), volt);
  DriveR.spin(fwd, to_volt(rightthrottle), volt);
}

/**
 * Tracking task to run in the background.
 */

int Drive::position_track_task(){
  chassis.position_track();
  return(0);
}
