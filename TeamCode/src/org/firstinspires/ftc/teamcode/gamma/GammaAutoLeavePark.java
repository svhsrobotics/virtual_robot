package org.firstinspires.ftc.teamcode.gamma;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.pedropathing.api.Paths.line;

/**
 * Gamma Auto, level 1: LEAVE and PARK.
 *
 * Scoring (manual Table 10-2): LEAVE = 3 points (not touching the wall at the end of AUTO),
 * PARK = 5 points (at least partly in our LOADING ZONE at the end of AUTO).
 * 8 points per robot. If both robots in the alliance do it, that's 16 = the SWARM ranking point.
 * No launcher needed, so this is the first autonomous Gamma can run in competition.
 *
 * FIELD COORDINATES (Pedro): inches, (0, 0) is the corner on the audience side of the RED wall.
 * x grows toward the blue side, y grows away from the audience. Heading 0 = facing the blue side,
 * 90 = facing away from the audience. Positions read from manual Figures 9-2 and 9-5 (TU03), so
 * check them against the real field before trusting them to the inch.
 *
 * RED vs BLUE: the BIOBUZZ field is the same for both alliances turned halfway around the center
 * (the blue LOADING ZONE is on the opposite wall, at the opposite end). Pedro calls this
 * "mirrorAroundPoint": every pose is spun 180 degrees around the field center (72, 72).
 * So we write the RED poses once and let Pedro work out the blue ones.
 */
@Autonomous(name = "Gamma: Auto Leave + Park", group = "Gamma")
public class GammaAutoLeavePark extends LinearOpMode {

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
        Pose start = p.of(9, 72, 0);      // touching our wall (robot is 18 in wide), facing the HIVE
        Pose offWall = p.of(26, 72, 0);   // about 8 in clear of the wall
        Pose park = p.of(14, 108, 0);     // partly in our LOADING ZONE, 5 in clear of the wall

        // Tell Pedro where the robot is starting. Pedro only knows what we tell it:
        // if the robot is really somewhere else, every path ends up in the wrong place.
        robot.follower.setPose(start);

        // Step 1: LEAVE. Drive straight off the wall, keeping the same heading.
        drivePath("1 Leave the wall", line(start, offWall).constant(start), 3.0);

        // Step 2: PARK. Drive to the LOADING ZONE.
        drivePath("2 Park in the LOADING ZONE", line(offWall, park).constant(offWall), 4.0);

        // Done: Pedro holds the robot in place until AUTO ends (it keeps updating).
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
