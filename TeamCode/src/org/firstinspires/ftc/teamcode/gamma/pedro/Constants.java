package org.firstinspires.ftc.teamcode.gamma.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.ThreeWheelConfig;
import com.pedropathing.revhub.localizers.ThreeWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Pedro Pathing settings for Gamma's robot.
 *
 * SIMULATOR VERSION: these numbers fit the simulator's "Gamma Bot". This is the ONE file that changes
 * when this code moves to the real robot. On the real robot:
 *   - drivetrainConfig: same motor names; directions come from Pedro's Mecanum tuner
 *   - localizerConfig: pod names become the names of the motor ports the pods plug into,
 *     and every number comes from Pedro's Three Wheel tuner
 *   - foresightConfig: every number comes from Pedro's Foresight tuner
 * Everything else (GammaRobot, the TeleOp, the autonomous) should move over unchanged.
 */
public class Constants {

    /*
     * DRIVETRAIN: which motor is which, and which way each one spins.
     * Names match Gamma's own Drivetrain class (note the capital B in "right_Back").
     */
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("left_front");
                c.backLeftName.set("left_back");
                c.frontRightName.set("right_front");
                c.backRightName.set("right_Back");
                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.manualBrakeMode.set(true);
            }
    );

    /*
     * LOCALIZER: the 3 odometry pods, which tell Pedro where the robot is.
     *
     * Pedro measures pod positions from the center of the robot, in inches:
     *   +X = toward the front, +Y = toward the LEFT side.
     * (GammaBot.java places the left pod 6.5 in left, the right pod 6.5 in right,
     *  and the sideways pod 3 in behind center.)
     *
     * Ticks to inches: a pod wheel 1.26 in across rolls PI x 1.26 = 3.958 in per turn, and the
     * simulator counts 1120 ticks per turn, so 1 tick = 3.958 / 1120 = 0.003534 in.
     */
    public static ThreeWheelConfig localizerConfig = new ThreeWheelConfig(
            c -> {
                c.leftEncoderName.set("pod_left");
                c.rightEncoderName.set("pod_right");
                c.strafeEncoderName.set("pod_strafe");
                c.leftPodY.set(6.5);
                c.rightPodY.set(-6.5);
                c.strafePodX.set(-3.0);
                c.forwardTicksToInches.set(0.0035343);
                c.strafeTicksToInches.set(0.0035343);
                c.turnTicksToRadians.set(0.0035343);
                // The left pod is mounted facing the other way, so its count goes DOWN when driving forward
                c.leftEncoderDirection.set(Encoder.REVERSE);
                c.rightEncoderDirection.set(Encoder.FORWARD);
                c.strafeEncoderDirection.set(Encoder.FORWARD);
            }
    );

    /*
     * FORESIGHT: how hard Pedro brakes and corrects. These are the simulator author's tuned numbers
     * for the MecDynamic Bot, which has the same physics as the Gamma Bot.
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.29);
                Controller secondaryTranslationalForward = Controller.proportional(0.11);
                Controller primaryTranslationalLateral = Controller.proportional(0.29);
                Controller secondaryTranslationalLateral = Controller.proportional(0.11);
                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
                c.coast.set(Controller.proportionalFeedforward(0.02));
                c.brake.set(Controller.proportionalFeedforward(0.017));
                c.headingFeedback.set(Controller.proportional(3.14));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.045, 0.0022));
                c.linearBrakeCoefficients.set(Matrix.diag(0.11, 0.14));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0017, 1.6e-4));
                c.maxAchievableForwardVelocity.set(49.9);
                c.maxAchievableStrafeVelocity.set(49.9);
                c.naturalForwardDeceleration.set(24.4);
                c.naturalStrafeDeceleration.set(24.4);
            }
    );

    /** Builds Pedro's follower: the pods ("where am I?"), the wheels ("how do I move?"), and Foresight ("how do I stay on the path?"). */
    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new ThreeWheelLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
