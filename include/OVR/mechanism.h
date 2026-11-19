#pragma once
#include "vex.h"

/**
 * Position-controlled mechanism, like a lift, arm, or wrist. A PID
 * runs in the background and holds the mechanism at its target, so
 * move_to() returns right away and the rest of your code keeps going.
 * Positions are in mechanism degrees (motor degrees times ratio).
 */

class Mechanism
{
public:
  motor_group motors;
  float ratio;
  float kp;
  float ki;
  float kd;
  float starti;
  float max_voltage;
  float settle_error;
  float kg = 0;
  bool is_arm = false;
  float target = 0;
  bool manual = true;
  vex::task control_task;
  vex::timer heartbeat;

  Mechanism(motor_group motors, float ratio, float kp, float ki, float kd, float starti, float max_voltage, float settle_error);

  void set_gravity(float kg, bool is_arm);

  float get_position();
  void set_position(float position);

  void move_to(float target);
  void move_to(float target, float timeout);
  void spin(float voltage);
  void hold();
  bool is_settled();

  void control_loop();
  static int control_loop_task(void* mechanism);
};
