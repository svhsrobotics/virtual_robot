package org.firstinspires.ftc.teamcode.gamma;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

/**
 * Gamma Auto, level 2: LAUNCH the 4 pre-loaded POLLEN, then PARK.
 *
 * What it teaches (one new Pedro idea per step):
 *   Step 1  a CURVED path, while the robot turns to FACE the HIVE  (curve + facingPoint)
 *   Step 2  stand still and launch                                   (Pedro holds the robot in place)
 *   Step 3  a straight path from wherever the robot really is        (follower.pose())
 *
 * Scoring (manual Table 10-2): LEAVE = 3, PARK = 5, plus points for every POLLEN that lands in the
 * HIVE. Rule G304: the robot starts touching exactly 4 POLLEN. Rule G407: never more than 4 at once.
 * Rule G402: in AUTO, stay on our own half of the field (red: x less than 72).
 *
 * THE LAUNCHER ISN'T BUILT YET. robot.launch() only writes a telemetry line, and the launch spot
 * below is a guess. When the launcher exists, the team tests from which spot it scores best and
 * changes launchSpot. The rest of this file stays the same.
 *
 * FIELD COORDINATES (Pedro): inches, (0, 0) is the corner on the audience side of the RED wall.
 * x grows toward the blue side, y grows away from the audience. Heading 0 = facing the blue side,
 * 90 = facing away from the audience. Same as GammaAutoLeavePark.
 *
 * RED vs BLUE: write the RED poses once; mirrorAroundPoint(72, 72) spins every pose halfway around
 * the field center to get the BLUE ones (the blue side is the red side turned around).
 */
@Autonomous(name = "Gamma: Auto Launch + Park", group = "Gamma")
public class GammaAutoLaunchPark extends LinearOpMode {

    // How long to stay still while launching. A guess until the launcher exists:
    // then replace the timer with "until the launcher says it's empty".
    static final double LAUNCH_SECONDS = 2.0;

    GammaRobot robot = new GammaRobot();
    ElapsedTime matchTimer = new ElapsedTime();
    boolean blue = false;
    String step = "Waiting for START";

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        // During INIT the drive team picks the alliance. Nothing moves here (rules G403/G404).
        while (opModeInInit()) {
            if (gamepad1.x) blue = true;
            if (gamepad1.b) blue = false;
            telemetry.addLine("Press X for BLUE, B for RED");
            telemetry.addData("Alliance", blue ? "BLUE" : "RED");
            telemetry.addLine("Place the robot at the start spot, then press START.");
            telemetry.update();
        }
        if (isStopRequested()) return;
        matchTimer.reset();

        // RED poses (x, y in inches, heading in degrees). For blue, Pedro spins each one around the center.
        PoseFactory p = blue ? PoseFactory.degrees().mirrorAroundPoint(72, 72) : PoseFactory.degrees();
        Pose start      = p.of(9, 72, 0);    // touching our wall (robot is 18 in wide), facing the HIVE
        Pose bend       = p.of(36, 72, 0);   // the curve bends toward this point (the robot never drives through it)
        Pose launchSpot = p.of(32, 96, 0);   // where we launch from: a GUESS until the launcher is tested
        Pose hive       = p.of(59, 72, 0);   // middle of OUR HIVE: the point the robot turns to face
        Pose park       = p.of(14, 108, 0);  // partly in our LOADING ZONE, 5 in clear of the wall

        // Tell Pedro where the robot is starting. Pedro only knows what we tell it.
        robot.follower.setPose(start);

        // Step 1: drive a curve to the launch spot, turning to face the HIVE the whole way.
        // Spin the launcher up now so it's at full speed when we arrive (saves AUTO time).
        robot.launcherSpinUp();
        drivePath("1 Curve to the launch spot, facing the HIVE",
                curve(start, bend, launchSpot).facingPoint(hive), 4.0);

        // Step 2: launch. The path is finished, so Pedro holds the robot still at its end point
        // (if another robot bumps us, Pedro pushes back to the spot).
        step = "2 Launch 4 POLLEN";
        robot.launch(4);
        ElapsedTime launchTimer = new ElapsedTime();
        while (opModeIsActive() && launchTimer.seconds() < LAUNCH_SECONDS) {
            robot.follower.update();   // keep calling update(), or Pedro stops holding
            showStatus();
        }
        robot.launcherStop();

        // Step 3: drive straight to PARK. Start the path from where the robot REALLY is
        // (follower.pose()), not where we planned it to be. Turn smoothly back to heading 0 on the way.
        Pose here = robot.follower.pose();
        drivePath("3 Park in the LOADING ZONE", line(here, park).linear(here, park), 4.0);

        // Done: Pedro holds the robot in place until AUTO ends.
        step = "Done: parked, holding position";
        while (opModeIsActive()) {
            robot.follower.update();
            showStatus();
        }
    }

    /**
     * Drive one path and come back when it's finished, OR when the time limit runs out,
     * OR when AUTO is stopped. The time limit means a robot that gets blocked can't use up all of AUTO.
     */
    void drivePath(String stepName, Path path, double timeLimitSeconds) {
        step = stepName;
        robot.follower.follow(path);
        ElapsedTime stepTimer = new ElapsedTime();
        while (opModeIsActive() && robot.follower.isBusy() && stepTimer.seconds() < timeLimitSeconds) {
            robot.follower.update();
            showStatus();
        }
        if (stepTimer.seconds() >= timeLimitSeconds) {
            telemetry.log().add(stepName + ": TIME LIMIT hit, moving on");
        }
    }

    void showStatus() {
        Pose pose = robot.follower.pose();
        telemetry.addData("Alliance", blue ? "BLUE" : "RED");
        telemetry.addData("Step", step);
        telemetry.addData("AUTO time used", "%.1f of 30 s", matchTimer.seconds());
        telemetry.addData("Position (in)", "x %.1f   y %.1f", pose.x(), pose.y());
        // Pedro reports heading from 0 to 360. Show -180 to 180 instead, so "a little clockwise" reads -5, not 355.
        double headingDeg = Math.toDegrees(pose.heading());
        if (headingDeg > 180) headingDeg -= 360;
        telemetry.addData("Heading (deg)", "%.0f", headingDeg);
        telemetry.update();
    }
}
