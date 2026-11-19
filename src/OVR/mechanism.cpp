#include "vex.h"

/**
 * Mechanism constructor.
 *
 * @param motors The motors that drive the mechanism.
 * @param ratio External ratio in input teeth/output teeth, same as the drive.
 * A 12-tooth gear on the motor driving a 60-tooth gear on the lift is 0.2.
 * @param kp Proportional constant.
 * @param ki Integral constant.
 * @param kd Derivative constant.
 * @param starti Maximum error in degrees to start integrating.
 * @param max_voltage Max voltage out of 12.
 * @param settle_error Error in degrees that counts as being there.
 */

Mechanism::Mechanism(motor_group motors, float ratio, float kp, float ki, float kd, float starti, float max_voltage, float settle_error) :
  motors(motors),
  ratio(ratio),
  kp(kp),
  ki(ki),
  kd(kd),
  starti(starti),
  max_voltage(max_voltage),
  settle_error(settle_error)
{};

/**
 * Adds a constant voltage that holds the mechanism up against gravity.
 * For an arm, the voltage is scaled by cos(position), so set the
 * position to 0 when the arm is level (horizontal).
 *
 * @param kg Volts needed to hold the mechanism still.
 * @param is_arm True for arms that swing, false for elevators and lifts that move straight up.
 */

void Mechanism::set_gravity(float kg, bool is_arm){
  this->kg = kg;
  this->is_arm = is_arm;
}

/**
 * @return Position in mechanism degrees.
 */

float Mechanism::get_position(){
  return(motors.position(deg)*ratio);
}

/**
 * Tells the mechanism where it is now, like set_position(0) when the
 * lift is all the way down at the start of the match.
 *
 * @param position Current position in mechanism degrees.
 */

void Mechanism::set_position(float position){
  motors.setPosition(position/ratio, deg);
  target = position;
}

/**
 * Starts moving to a target and returns right away. The mechanism
 * keeps holding the target until it's given a new one. The PID task
 * starts here, and restarts if competition control stopped it between
 * autonomous and driver control.
 *
 * @param target Desired position in mechanism degrees.
 */

void Mechanism::move_to(float target){
  this->target = target;
  manual = false;
  if(heartbeat.time(msec) > 50){
    heartbeat.clear();
    control_task = task(control_loop_task, (void*)this);
  }
}

/**
 * Moves to a target and waits until it gets there or times out.
 *
 * @param target Desired position in mechanism degrees.
 * @param timeout Time in ms after which to stop waiting.
 */

void Mechanism::move_to(float target, float timeout){
  move_to(target);
  float time_waited = 0;
  while(!is_settled() && time_waited < timeout){
    task::sleep(10);
    time_waited += 10;
  }
}

/**
 * Spins the mechanism at a voltage, turning the PID off until the
 * next move_to() or hold(). Use it for manual driver control.
 *
 * @param voltage Voltage out of 12.
 */

void Mechanism::spin(float voltage){
  manual = true;
  motors.spin(fwd, voltage, volt);
}

/**
 * Holds the mechanism where it is now, like after letting go of a manual button.
 */

void Mechanism::hold(){
  move_to(get_position());
}

/**
 * @return Whether the mechanism is within settle_error of its target.
 */

bool Mechanism::is_settled(){
  return(fabs(target-get_position()) < settle_error);
}

/**
 * Background PID loop that holds the target.
 */

void Mechanism::control_loop(){
  PID mechanismPID(0, kp, ki, kd, starti);
  while(1){
    if(!manual){
      float gravity = kg;
      if(is_arm){ gravity = kg*cos(to_rad(get_position())); }
      float output = mechanismPID.compute(target-get_position()) + gravity;
      motors.spin(fwd, clamp(output, -max_voltage, max_voltage), volt);
    }
    heartbeat.clear();
    task::sleep(10);
  }
}

int Mechanism::control_loop_task(void* mechanism){
  ((Mechanism*)mechanism)->control_loop();
  return(0);
}
