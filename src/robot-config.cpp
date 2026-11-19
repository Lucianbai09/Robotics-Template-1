#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;
controller Controller = controller(primary);

//The motor constructor takes motors as (port, ratio, reversed), so for example
//motor LeftFront = motor(PORT1, ratio6_1, false);
//5.5W motors have no cartridge, so they are just (port, reversed):
//motor LeftMiddle = motor(PORT2, false);

//Add your devices below, and don't forget to do the same in robot-config.h:

// EXAMPLE Override devices. Change the ports to match your robot, and delete
// anything you don't have (and its lines in main.cpp and autons.cpp).
// R10 allows 88W of motors in total and R11 allows at most 55W on the drive,
// so e.g. a 4x11W + 2x5.5W drive (55W) leaves 2x11W + 2x5.5W (33W) for these.
motor LiftL = motor(PORT11, ratio18_1, false);
motor LiftR = motor(PORT12, ratio18_1, true);
motor WristMotor = motor(PORT13, false);
motor Intake = motor(PORT14, false);
digital_out Claw = digital_out(Brain.ThreeWirePort.A);
optical ColorSensor = optical(PORT15);
vex::distance LeftDistance = vex::distance(PORT16);
vex::distance BackDistance = vex::distance(PORT17);

// Mechanism(motors, ratio, kp, ki, kd, starti, max_voltage, settle_error)
// The ratio works like the drive's: input teeth/output teeth.
Mechanism Lift(motor_group(LiftL, LiftR), 1, 0.15, 0, 0.5, 0, 12, 5);
Mechanism Wrist(motor_group(WristMotor), 1, 0.1, 0, 0.3, 0, 8, 5);

void vexcodeInit( void ) {
  ColorSensor.setLight(ledState::on);
  ColorSensor.setLightPower(100, percent);
}
