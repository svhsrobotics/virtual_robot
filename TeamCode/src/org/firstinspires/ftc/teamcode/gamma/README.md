# Gamma (19831) Pedro Pathing examples for the simulator

Example code for learning Pedro Pathing 3 on a simulated version of Gamma's BIOBUZZ robot.
It is not the team's robot code: read it, try it, and borrow what makes sense.

## The simulated robot: "Gamma Bot"
Pick **Gamma Bot** in the simulator's Configuration dropdown. It has the parts Gamma's code uses:

| Part | Name in code |
|---|---|
| 4 mecanum drive motors | `left_front`, `right_front`, `left_back`, `right_Back` |
| 3 odometry pods (simulator only names) | `pod_left`, `pod_right`, `pod_strafe` |
| 2 intake bar servos (pink arms on screen) | `left_intake`, `right_intake` |
| Intake wheel motor (roller turns green / red) | `intake` |

The blue rectangles on the robot drawing are the pods. Their positions are placeholders until the real robot is measured.

## The files
| File | What it is |
|---|---|
| `pedro/Constants.java` | Pedro's settings. **The only file that changes for the real robot** |
| `GammaRobot.java` | The one class that knows the hardware. Every OpMode uses it |
| `GammaTeleOp.java` | Drive through Pedro, intake bar on left bumper, intake wheels on the triggers |
| `GammaPodTest.java` | Shows the 3 raw pod counts next to Pedro's position. Predict, then drive |
| `GammaAutoLeavePark.java` | Autonomous level 1: leave the wall and park in the LOADING ZONE (3 + 5 points) |
| `GammaManualDrive.java` | Manual driving with no path library: read the sticks, mix them into 4 wheel powers, scale down, send. Built like FIRST's own sample (BasicOmniOpMode_Linear) |
| `GammaAutoLaunchPark.java` | Autonomous level 2: curve to a launch spot while turning to face the HIVE, launch the 4 pre-loaded POLLEN (pretend: no launcher yet), then park |

## Running the autonomous
1. Configuration: **Gamma Bot**. OpMode: **Gamma: Auto Leave + Park**.
2. Place the robot at the red start spot: against the red (left) wall, on the line halfway up the field (between tile rows 3 and 4), facing right. Left-click right next to the left wall on that line (the simulator stops the robot at the wall), then right-click to the right of the robot to aim it.
3. Press **INIT**. Press B for red or X for blue on the gamepad.
4. Press **START**. Watch the telemetry: step name, AUTO time used, and position.

Try it: place the robot somewhere else and run it again. Pedro only knows what `setPose` tells it.
