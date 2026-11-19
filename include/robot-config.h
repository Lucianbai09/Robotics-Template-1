using namespace vex;

extern brain Brain;
extern controller Controller;

//To set up a motor called LeftFront here, you'd use
//extern motor LeftFront;

//Add your devices below, and don't forget to do the same in robot-config.cpp:

// EXAMPLE Override devices. Change or delete these to match your robot.
extern motor LiftL;
extern motor LiftR;
extern motor WristMotor;
extern motor Intake;
extern digital_out Claw;
extern optical ColorSensor;
extern vex::distance LeftDistance;
extern vex::distance BackDistance;

extern Mechanism Lift;
extern Mechanism Wrist;

// Lift presets in lift degrees. These are placeholders, measure your own
// by printing Lift.get_position() with the lift at each goal.
const float LIFT_DOWN = 0;
const float LIFT_SHORT_GOAL = 250;
const float LIFT_ALLIANCE_GOAL = 250;
const float LIFT_TALL_GOAL = 600;

// Wrist presets in wrist degrees, for flipping a Pin end over end.
const float WRIST_UP = 0;
const float WRIST_FLIPPED = 180;

void  vexcodeInit( void );
