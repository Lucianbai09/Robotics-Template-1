# OVR Template

A VEXcode V5 C++ template for the 2026-2027 V5RC game **Override**. It's built on [JAR-Template](https://github.com/JacksonAreaRobotics/JAR-Template), so every JAR function works the same way, with the features from LemLib, EZ-Template and RW-Template that matter most for Override added on top.

## What's added on top of JAR

| Feature | What it's for |
|---|---|
| Async motions: `run_async()`, `wait_until_traveled()`, `wait_until_turned()`, `wait_until_done()`, `cancel_motion()` | Run the lift or intake while the drive moves, or stop a drive early when a sensor sees a Pin |
| Motion queueing | A motion called while a background one is running waits its turn instead of fighting it |
| Drive slew (`drive_slew`) | Limits acceleration so a 55W drive (R11) doesn't slip or spike current |
| Stall exit (`stall_time`) | Ends a motion when the robot is pushed up against a Goal, Loader or robot, instead of waiting out the timeout |
| Auton mirroring (`mirror`) | Write an auton once and run it from either starting tile on your side |
| Distance sensor wall resets (`reset_from_wall()`) | Fixes odom drift during the 1-minute skills run |
| `Mechanism` class | Background PID with presets and gravity hold, for lifts, arms and wrists |
| Color sensing (`get_color()`) | Tells red, blue and yellow Pin halves apart |
| Auton selector | Named autons on the Brain and controller, alliance color, mirror per entry |
| Driver extras | Joystick curve, steering that still works at full throttle, Endgame rumble, drive temperature on the controller |

### JAR bugs fixed
- Odom lost a bit of position every time the heading wrapped past 0/360. In a simulated 5-spin test JAR's odom ended 0.9" to 4.6" off. This version ends within 0.01".
- `drive_to_pose()` could finish facing backwards. It now always ends front-first at the angle you ask for.
- Swings ignored `swing_max_voltage` (they used `turn_max_voltage`).
- Joysticks maxed out at about 79% stick, because `value()` (out of 127) was treated as a percent.
- `drive_distance()` now uses the forward tracking wheel when you have one, since it doesn't slip.
- Calling `set_coordinates()` twice started a second odom task.
- Competition control may stop tasks between autonomous and driver control, and the auton that's running always gets cut off. Motions, odom and mechanism PIDs now recover on their own instead of locking up the next match.

## Setup

1. Install VS Code and the **VEX Robotics** extension. Open this folder, or import the zip with the extension's Import option.
2. **robot-config.cpp / .h:** add your motors and sensors. The example Override devices (lift, wrist, claw, intake, color and distance sensors) are there to show the setup, so change their ports or delete them.
3. **main.cpp:** fill in the `Drive chassis(...)` constructor like JAR: drive setup, motor groups, IMU port, wheel size, gear ratio, and tracking wheels if you have them.
   - R11 allows at most 55W on the drive (5x11W, 4x11W + 2x5.5W, or 10x5.5W). R10 allows 88W of motors in total.
   - Motor groups report their *first* motor's position, so if you mix 11W and 5.5W motors, put one that matches your external ratio first.
4. **Tune:** run Drive Test and Turn Test from the selector and adjust `default_constants()` in autons.cpp until they stop on target. If you have odom, use Odom Test to check that the coordinates match real life.
5. **Write autons** in autons.cpp, declare them in autons.h, and add them to the `autons[]` list in main.cpp.

## Auton selector

On the Brain screen, tap the left third for the previous auton, the middle to switch alliance color, and the right third for the next one. With the robot disabled, the controller's Left/Right arrows change the auton and Up switches the color. Each entry has a mirror flag:

```cpp
auton autons[] = {
  {"Left AWP", left_awp, false},
  {"Right AWP", left_awp, true},  // same code, mirrored
  {"Skills", skills, false},
};
```

Without a competition switch, hold Left + Right in driver control to run the selected auton.

## Using the new features

```cpp
// Background motion, with the intake timed to it
chassis.run_async([](){ chassis.drive_distance(36); });
chassis.wait_until_traveled(12);
Intake.spin(fwd, 12, volt);
waitUntil(!chassis.is_moving() || get_color(ColorSensor) != COLOR_NONE);
chassis.cancel_motion();     // stop early once a Pin is in
chassis.wait_until_done();

// Mechanisms move in the background, so they don't hold up the drive
Lift.move_to(LIFT_TALL_GOAL);        // returns right away
Lift.move_to(LIFT_TALL_GOAL, 1000);  // waits until there, or 1 second
Lift.set_gravity(1.2, false);        // volts to hold it up (true for arms)

// Push into a Goal without waiting out the timeout
chassis.stall_time = 200;
chassis.drive_distance(10);
chassis.stall_time = 0;

// Chaining, same as JAR: a min voltage carries one move into the next
chassis.drive_min_voltage = 6;
chassis.drive_to_point(0, 24);
chassis.drive_min_voltage = 0;
chassis.drive_to_point(24, 48);

// Skills: field coordinates, (0, 0) at the center and walls at +-70.25 in
chassis.set_coordinates(-58, -40, 90);
chassis.reset_from_wall(LeftDistance, -7, 0, 270, 6); // sensor 7" left of center, facing left, max 6" jump
```

`reset_from_wall()` arguments: the sensor, inches right of the robot's center (negative is left), inches in front (negative is behind), the way it faces (0 front, 90 right, 180 back, 270 left), and an optional max jump. The robot must be within 15° of square to the wall, and the beam must hit the wall itself, not a Toggle (center of each wall), Loader or robot.

## Override notes
- **Match:** 15s autonomous, then 1:45 driver control. The last 10 seconds are the Endgame (SG12): robots in the Midfield have to be back to about starting height. Driver control rumbles at 15s and 10s left, and the example B button stows the lift.
- **AWP (SC8):** by the end of auton, 6 of your Pin halves Scored on your side of the Autonomous Line, in 2+ Goals with 2+ halves each, and neither robot touching the perimeter. Events that qualify for Worlds need 7 halves in 3 Goals.
- **Field coordinates:** the comment above `example_skills()` lists every Goal, Toggle and red Loader in inches, converted from the VEX GPS field map. Check them against your own field.
- **Skills:** Match Loads only go through the red Loaders.

Rules can change, so check the current game manual. This summary is from the manual and Q&A as of November 2026.

## Credits
Built on JAR-Template by Josh (2775V, Jackson Area Robotics), MIT license. Ideas from LemLib (async motions, motion chaining), EZ-Template (mirroring, exit conditions) and RW-Template (stall exit, distance resets).
