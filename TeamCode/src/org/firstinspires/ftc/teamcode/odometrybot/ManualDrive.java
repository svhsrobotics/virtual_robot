package org.firstinspires.ftc.teamcode.odometrybot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Manual mecanum driving: read the sticks, work out 4 wheel powers, send them. No path library.
 * EXAMPLE CODE FOR LEARNING (simulator), built the same way as FIRST's own sample
 * "BasicOmniOpMode_Linear" (in the SDK's external/samples folder), with the Mecanum Odometry Bot's motor names.
 *
 * Sticks (FIRST's sample layout):
 *   left stick up/down    = drive forward and back   (axial)
 *   left stick left/right = slide sideways            (lateral)
 *   right stick left/right = turn                     (yaw)
 *
 * One file on purpose, so the whole idea fits on one screen. In team code the motors live in
 * the shared hardware class (RobotHardware), and a path library like Pedro does this math for you.
 */
@TeleOp(name = "Manual Drive", group = "Mecanum Odometry Bot")
public class ManualDrive extends LinearOpMode {

    DcMotor leftFront, rightFront, leftBack, rightBack;

    @Override
    public void runOpMode() {
        // INIT: find the 4 motors by their configuration names. Nothing moves yet.
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_Back");

        // The left motors face the other way, so reverse them: then "positive power" means
        // "roll forward" on every wheel.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Ready. Press START.");
        telemetry.update();
        waitForStart();

        // LOOP: many times a second, until STOP.
        while (opModeIsActive()) {
            // 1. READ the sticks. Pushing up gives a NEGATIVE number, so flip it.
            double axial   = -gamepad1.left_stick_y;   // forward (+) and back (-)
            double lateral =  gamepad1.left_stick_x;   // slide right (+) and left (-)
            double yaw     =  gamepad1.right_stick_x;  // turn clockwise (+) and counterclockwise (-)

            // 2. MIX them into one power per wheel. Each wheel's rollers push at an angle,
            //    so each motion adds to some wheels and subtracts from others.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // 3. SCALE DOWN if any power is over 1.0 (the most a motor can take).
            //    Dividing all 4 by the biggest keeps their balance, so the robot still goes
            //    the way the sticks say, just at the top speed it can manage.
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // 4. SEND the powers to the wheels.
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);

            // 5. SHOW what the code is doing on the Driver Station.
            telemetry.addData("Sticks", "axial %.2f  lateral %.2f  yaw %.2f", axial, lateral, yaw);
            telemetry.addData("Front", "left %.2f  right %.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back", "left %.2f  right %.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }

        // After STOP: make sure nothing keeps moving (rule G404).
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
