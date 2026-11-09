package org.firstinspires.ftc.teamcode.gamma;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.gamma.pedro.Constants;

/**
 * Gamma Pod Test: shows the raw count from each odometry pod, next to where Pedro thinks the robot is.
 *
 * Use it to learn how 3 counting wheels become a position, and (on the real robot) to check that
 * every pod is plugged in, named right, and counting the right way.
 *
 * TRY THIS, predicting each answer before you drive:
 *   1. Drive straight forward. Which counts change? Which way?
 *   2. Drive straight sideways to the left. Which counts change now?
 *   3. Turn in place counterclockwise. What do the left and right pods do?
 *   4. Why does the left pod count DOWN when the robot drives forward? (Hint: look at how it's mounted.)
 *
 * The pod names come from Constants.java, so this same file works on the real robot once
 * Constants.java has the real names.
 */
@TeleOp(name = "Gamma: Pod Test", group = "Gamma")
public class GammaPodTest extends OpMode {

    GammaRobot robot = new GammaRobot();

    DcMotorEx podLeft;
    DcMotorEx podRight;
    DcMotorEx podStrafe;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);

        // Read each pod by the name Pedro uses for it
        podLeft = hardwareMap.get(DcMotorEx.class, Constants.localizerConfig.leftEncoderName.get());
        podRight = hardwareMap.get(DcMotorEx.class, Constants.localizerConfig.rightEncoderName.get());
        podStrafe = hardwareMap.get(DcMotorEx.class, Constants.localizerConfig.strafeEncoderName.get());

        telemetry.addLine("Press START, then drive with gamepad 1 and watch the counts.");
    }

    @Override
    public void loop() {
        // Same driving as Gamma: TeleOp (see that file for why the signs are flipped)
        robot.follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        robot.follower.update();

        int left = podLeft.getCurrentPosition();
        int right = podRight.getCurrentPosition();
        int strafe = podStrafe.getCurrentPosition();
        double inchesPerTick = Constants.localizerConfig.forwardTicksToInches.get();

        telemetry.addLine("--- Raw pod counts (ticks) ---");
        telemetry.addData("Left pod", "%d   (%.1f in)", left, left * inchesPerTick);
        telemetry.addData("Right pod", "%d   (%.1f in)", right, right * inchesPerTick);
        telemetry.addData("Sideways pod", "%d   (%.1f in)", strafe, strafe * inchesPerTick);

        Pose pose = robot.follower.pose();
        telemetry.addLine("--- What Pedro works out from them ---");
        telemetry.addData("Position (in)", "x %.1f   y %.1f", pose.x(), pose.y());
        // Pedro reports heading from 0 to 360. Show -180 to 180 instead, so "a little clockwise" reads -5, not 355.
        double headingDeg = Math.toDegrees(pose.heading());
        if (headingDeg > 180) headingDeg -= 360;
        telemetry.addData("Heading (deg)", "%.0f", headingDeg);
        telemetry.update();
    }
}
