package org.firstinspires.ftc.teamcode.odometrybot;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.odometrybot.pedro.Constants;

/**
 * RobotHardware: the ONE place that knows about the robot's hardware. Every OpMode (TeleOp, autonomous, tests)
 * creates a RobotHardware and calls init(). If a motor name or a servo position changes, it changes here only.
 *
 * EXAMPLE CODE FOR LEARNING (simulator). Hardware names come from one of our program's team robots (Team 19831 Gamma's own code)
 * (Haisen's Drivetrain and Intake classes, Nov 2026).
 *
 * What it holds:
 *   - follower: Pedro's driver. It owns the 4 drive motors and the 3 odometry pods.
 *     Nothing else should ever set drive motor power, or the wheels get two bosses.
 *   - the intake: 2 servos that raise and lower the intake bar, and 1 motor that spins the intake wheels
 *   - the launcher: not built yet, so its methods only report on telemetry
 */
public class RobotHardware {

    // Intake bar servo positions (placeholders: find the real values on the robot)
    public static final double INTAKE_RAISED = 0.0;
    public static final double INTAKE_LOWERED = 1.0;

    // Intake wheel power: positive pulls balls in, negative pushes them out
    public static final double INTAKE_IN_POWER = 0.8;
    public static final double INTAKE_OUT_POWER = -0.8;

    public Follower follower;

    private Servo leftIntake;
    private Servo rightIntake;
    private DcMotor intakeMotor;
    private Telemetry telemetry;

    /**
     * Find every part by its configuration name. Call this in INIT.
     *
     * Rule G403/G404: nothing may move during INIT if it runs during the gap between AUTO and TELEOP.
     * So init() only FINDS the parts. It never sets a servo position or a motor power.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        follower = Constants.create(hardwareMap);

        leftIntake = hardwareMap.get(Servo.class, "left_intake");
        rightIntake = hardwareMap.get(Servo.class, "right_intake");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        // The two intake servos face each other, so one is reversed to make them move together
        leftIntake.setDirection(Servo.Direction.FORWARD);
        rightIntake.setDirection(Servo.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    /* ---------------- INTAKE ---------------- */

    public void intakeDown() {
        leftIntake.setPosition(INTAKE_LOWERED);
        rightIntake.setPosition(INTAKE_LOWERED);
    }

    public void intakeUp() {
        leftIntake.setPosition(INTAKE_RAISED);
        rightIntake.setPosition(INTAKE_RAISED);
    }

    /** Positive power pulls balls in, negative pushes them out, 0 stops. */
    public void runIntake(double power) {
        intakeMotor.setPower(power);
    }

    /* ---------------- LAUNCHER (not built yet) ---------------- */
    // These methods exist now so autonomous steps can already call them.
    // When the launcher is built, only the inside of these methods changes.

    public void launcherSpinUp() {
        telemetry.addLine("Launcher: not built yet (would spin up here)");
    }

    public void launch(int howMany) {
        telemetry.addLine("Launcher: not built yet (would launch " + howMany + " here)");
    }

    public void launcherStop() {
        telemetry.addLine("Launcher: not built yet (would stop here)");
    }
}
