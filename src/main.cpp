#include "vex.h"

using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             OVR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
ZERO_TRACKER_NO_ODOM,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".
//Override (R11) allows at most 55W on the drive, like 5x11W, 4x11W + 2x5.5W, or 10x5.5W.
//Positions are read from the first motor in each group, so put a motor that matches the external ratio below first.

//Left Motors:
motor_group(),

//Right Motors:
motor_group(),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT1,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.6,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:
PORT1,     -PORT2,

//LB:      //RB:
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
3,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
-2,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
1,

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5

);

/*---------------------------------------------------------------------------*/
/*                              Auton Selector                               */
/*                                                                           */
/*  Add your autons to the list below. The name shows on the Brain and the   */
/*  controller. Setting mirror to true runs the same auton flipped left to   */
/*  right, so one auton covers both starting tiles on your side.             */
/*---------------------------------------------------------------------------*/

struct auton {
  const char* name;
  void (*routine)();
  bool mirror;
};

auton autons[] = {
  {"Example Left", example_auton, false},
  {"Example Right", example_auton, true},
  {"Skills", example_skills, false},
  {"Drive Test", drive_test, false},
  {"Turn Test", turn_test, false},
  {"Swing Test", swing_test, false},
  {"Full Test", full_test, false},
  {"Odom Test", odom_test, false},
  {"Tank Odom Test", tank_odom_test, false},
  {"Holo Odom Test", holonomic_odom_test, false},
  {"Async Example", async_example, false},
};

int auton_count = sizeof(autons)/sizeof(autons[0]);
int current_auton_selection = 0;
bool auto_started = false;
object_color alliance = COLOR_RED;

/**
 * Draws the selector on the Brain screen. Tap the left third for the
 * previous auton, the middle to switch alliance color, and the right
 * third for the next auton.
 */

void draw_selector(){
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(5, 20, "OVR Template v1.0");
  Brain.Screen.printAt(5, 50, "Auton: %s", autons[current_auton_selection].name);
  Brain.Screen.printAt(5, 80, "Alliance: %s", color_name(alliance));
  Brain.Screen.printAt(5, 110, "Battery: %d%%   Heading: %.1f", (int)Brain.Battery.capacity(), chassis.get_absolute_heading());
  if(chassis.Gyro.isCalibrating()){
    Brain.Screen.printAt(5, 140, "Calibrating IMU, keep the robot still");
  }
  Brain.Screen.drawRectangle(0, 180, 159, 59);
  Brain.Screen.drawRectangle(160, 180, 159, 59);
  Brain.Screen.drawRectangle(320, 180, 159, 59);
  Brain.Screen.printAt(45, 215, "< PREV");
  Brain.Screen.printAt(195, 215, "COLOR");
  Brain.Screen.printAt(355, 215, "NEXT >");
}

/**
 * Function before autonomous. It calibrates the IMU and runs the auton
 * selector until autonomous starts. While the robot is disabled, the
 * controller can also pick: Left/Right arrows change the auton and Up
 * switches alliance color.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();
  chassis.Gyro.calibrate();

  int last_selection = -1;
  object_color last_alliance = COLOR_NONE;
  int redraw_time = 0;
  while(!auto_started){
    if(Brain.Screen.pressing()){
      int x = Brain.Screen.xPosition();
      while(Brain.Screen.pressing()) { task::sleep(10); }
      if(x < 160){ current_auton_selection--; }
      else if(x < 320){ alliance = (alliance == COLOR_RED) ? COLOR_BLUE : COLOR_RED; }
      else { current_auton_selection++; }
    }
    if(!Competition.isEnabled()){
      if(Controller.ButtonLeft.pressing()){
        while(Controller.ButtonLeft.pressing()) { task::sleep(10); }
        current_auton_selection--;
      }
      if(Controller.ButtonRight.pressing()){
        while(Controller.ButtonRight.pressing()) { task::sleep(10); }
        current_auton_selection++;
      }
      if(Controller.ButtonUp.pressing()){
        while(Controller.ButtonUp.pressing()) { task::sleep(10); }
        alliance = (alliance == COLOR_RED) ? COLOR_BLUE : COLOR_RED;
      }
    }
    current_auton_selection = (current_auton_selection + auton_count) % auton_count;

    bool changed = current_auton_selection != last_selection || alliance != last_alliance;
    if(changed || redraw_time >= 500){
      draw_selector();
      redraw_time = 0;
    }
    if(changed && !Competition.isEnabled()){
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("%s", autons[current_auton_selection].name);
      Controller.Screen.setCursor(2, 1);
      Controller.Screen.print("%s", color_name(alliance));
    }
    last_selection = current_auton_selection;
    last_alliance = alliance;
    redraw_time += 20;
    task::sleep(20);
  }
}

/**
 * Runs the selected auton. Waits for the IMU to finish calibrating,
 * resets the constants, and sets mirror for the selected entry.
 */

void run_selected_auton(){
  while(chassis.Gyro.isCalibrating()) { task::sleep(10); }
  chassis.stop_motions();
  default_constants();
  chassis.mirror = autons[current_auton_selection].mirror;
  autons[current_auton_selection].routine();
}

/**
 * Auton function, which runs the selected auton. The first auton in the
 * list is the default, and will run if the selector goes untouched.
 */

void autonomous(void) {
  auto_started = true;
  run_selected_auton();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // A motion cut off at the end of autonomous never finished, so clear it out,
  // and make sure the mechanism PIDs are running and holding their targets.
  chassis.stop_motions();
  Lift.move_to(Lift.target);
  Wrist.move_to(Wrist.target);
  timer match_timer;
  bool claw_was_pressed = false;
  bool wrist_was_pressed = false;
  bool warned_15 = false;
  bool warned_10 = false;
  int screen_time = 0;

  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    //Replace this line with chassis.control_tank(); for tank drive
    //or chassis.control_holonomic(); for holo drive.
    chassis.control_arcade();

    // EXAMPLE Override controls. Change them to match your robot.
    // L1/L2 move the lift by hand, and it holds wherever you let go.
    if(Controller.ButtonL1.pressing()){ Lift.spin(12); }
    else if(Controller.ButtonL2.pressing()){ Lift.spin(-12); }
    else if(Lift.manual){ Lift.hold(); }

    // Arrows are lift presets. B stows everything, which also gets the
    // robot back to starting height for the Endgame (SG12).
    if(Controller.ButtonUp.pressing()){ Lift.move_to(LIFT_TALL_GOAL); }
    if(Controller.ButtonRight.pressing()){ Lift.move_to(LIFT_SHORT_GOAL); }
    if(Controller.ButtonDown.pressing()){ Lift.move_to(LIFT_DOWN); }
    if(Controller.ButtonB.pressing()){
      Lift.move_to(LIFT_DOWN);
      Wrist.move_to(WRIST_UP);
    }

    // A opens and closes the claw, X flips the Pin with the wrist.
    if(Controller.ButtonA.pressing() && !claw_was_pressed){ Claw.set(!Claw.value()); }
    claw_was_pressed = Controller.ButtonA.pressing();
    if(Controller.ButtonX.pressing() && !wrist_was_pressed){
      Wrist.move_to(Wrist.target == WRIST_UP ? WRIST_FLIPPED : WRIST_UP);
    }
    wrist_was_pressed = Controller.ButtonX.pressing();

    // R1/R2 run the intake in and out.
    if(Controller.ButtonR1.pressing()){ Intake.spin(fwd, 12, volt); }
    else if(Controller.ButtonR2.pressing()){ Intake.spin(reverse, 12, volt); }
    else { Intake.stop(); }

    // Driver Control is 1:45. Rumble with 15 seconds left, and again when
    // the 10 second Endgame starts.
    float time_left = 105 - match_timer.time(sec);
    if(time_left < 15 && !warned_15){ Controller.rumble("."); warned_15 = true; }
    if(time_left < 10 && !warned_10){ Controller.rumble("-"); warned_10 = true; }

    // Time left and drive temperature on the controller, to catch hot 5.5W motors.
    screen_time += 20;
    if(screen_time >= 500){
      screen_time = 0;
      Controller.Screen.setCursor(3, 1);
      Controller.Screen.print("%3.0fs  Drive %2.0fC   ", time_left, chassis.DriveL.temperature(celsius));
    }

    // Testing autons without a competition switch: hold Left and Right.
    if(!Competition.isFieldControl() && !Competition.isCompetitionSwitch() && Controller.ButtonLeft.pressing() && Controller.ButtonRight.pressing()){
      run_selected_auton();
      chassis.stop_motions();
    }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
