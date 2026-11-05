package org.firstinspires.ftc.teamcode.gamma;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Gamma TeleOp, driving through Pedro.
 *
 * Controls (gamepad 1):
 *   left stick        drive (forward, back, sideways)
 *   right stick X     turn
 *   left bumper       hold to lower the intake bar (it raises when you let go)
 *   right trigger     intake wheels pull in
 *   left trigger      intake wheels push out
 *
 * Why drive through Pedro in TeleOp? Pedro keeps tracking where the robot is the whole match, so later
 * the drivers can press a button and let Pedro drive to a launch spot.
 */
@TeleOp(name = "Gamma: TeleOp", group = "Gamma")
public class GammaTeleOp extends OpMode {

    GammaRobot robot = new GammaRobot();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        telemetry.addLine("Ready. Nothing moves until START.");
    }

    @Override
    public void loop() {
        // 1. Driving: forward, sideways, turn. Pedro's manual() is robot-centric:
        //    "forward" means the way the robot is facing.
        //    Why the minus signs: Pedro counts sideways as positive to the LEFT and turning as positive
        //    COUNTERCLOCKWISE, but pushing a stick right gives a positive number. Flipping the sign makes
        //    "stick right" mean "go right" and "turn right". (Pushing a stick forward gives a negative
        //    number, so forward gets a minus sign too.)
        robot.follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        // 2. Intake bar
        if (gamepad1.left_bumper) {
            robot.intakeDown();
        } else {
            robot.intakeUp();
        }

        // 3. Intake wheels
        if (gamepad1.right_trigger > 0.2) {
            robot.runIntake(GammaRobot.INTAKE_IN_POWER);
        } else if (gamepad1.left_trigger > 0.2) {
            robot.runIntake(GammaRobot.INTAKE_OUT_POWER);
        } else {
            robot.runIntake(0);
        }

        // 4. Pedro must update every loop, or nothing drives and the position stops updating
        robot.follower.update();

        // 5. Telemetry: where Pedro thinks the robot is
        Pose pose = robot.follower.pose();
        telemetry.addData("Position (in)", "x %.1f   y %.1f", pose.x(), pose.y());
        telemetry.addData("Heading (deg)", "%.0f", Math.toDegrees(pose.heading()));
        telemetry.update();
    }
}
