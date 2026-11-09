# Mecanum Odometry Bot: Pedro Pathing examples for the simulator

Example code for learning Pedro Pathing 3 on a simulated mecanum robot with 3 odometry pods and an intake.
It's for any team: read it, try it, and borrow what makes sense.

The robot is modeled on one of our program's BIOBUZZ robots (Team 19831 Gamma), so the motor and servo names match that team's code. A team with different names changes them in two places: `RobotHardware.java` and `pedro/Constants.java`.

## The simulated robot: "Mecanum Odometry Bot"
Pick **Mecanum Odometry Bot** in the simulator's Configuration dropdown. It has these parts:

| Part | Name in code |
|---|---|
| 4 mecanum drive motors | `left_front`, `right_front`, `left_back`, `right_Back` |
| 3 odometry pods (simulator only names) | `pod_left`, `pod_right`, `pod_strafe` |
| 2 intake bar servos (pink arms on screen) | `left_intake`, `right_intake` |
| Intake wheel motor (roller turns green / red) | `intake` |

The blue rectangles on the robot drawing are the pods. Their positions are placeholders until a real robot is measured.

## The files
| File | Driver Station name | What it is |
|---|---|---|
| `pedro/Constants.java` | | Pedro's settings. **The only file that changes for a real robot** |
| `RobotHardware.java` | | The one class that knows the hardware. Every OpMode uses it |
| `ManualDrive.java` | Manual Drive | Manual driving with no path library: read the sticks, mix them into 4 wheel powers, scale down, send. Built like FIRST's own sample (BasicOmniOpMode_Linear) |
| `PedroTeleOp.java` | Pedro: TeleOp | Drive through Pedro, intake bar on left bumper, intake wheels on the triggers |
| `PodTest.java` | Pod Test | Shows the 3 raw pod counts next to Pedro's position. Predict, then drive |
| `AutoLeavePark.java` | Pedro: Auto Leave + Park | Autonomous Level 1: leave the wall and park in the LOADING ZONE (3 + 5 points) |
| `AutoLaunchPark.java` | Pedro: Auto Launch + Park | Autonomous Level 2: curve to a launch spot while turning to face the HIVE, launch the 4 pre-loaded POLLEN (pretend: no launcher yet), then park |

## Running the autonomous
1. Configuration: **Mecanum Odometry Bot**. OpMode: **Pedro: Auto Leave + Park**.
2. Place the robot at the red start spot: against the red (left) wall, on the line halfway up the field (between tile rows 3 and 4), facing right. Left-click right next to the left wall on that line (the simulator stops the robot at the wall), then right-click to the right of the robot to aim it.
3. Press **INIT**. Press B for red or X for blue on the gamepad.
4. Press **START**. Watch the telemetry: step name, AUTO time used, and position.

Try it: place the robot somewhere else and run it again. Pedro only knows what `setPose` tells it.
