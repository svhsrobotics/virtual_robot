package virtual_robot.robots.classes;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.fxml.FXML;
import javafx.scene.paint.Color;
import javafx.scene.shape.Rectangle;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import virtual_robot.config.Config;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.Filters;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;

/**
 * Mecanum Odometry Bot: a mecanum robot with 3 odometry pods and an intake, for learning Pedro Pathing.
 * It is modeled on one of our program's BIOBUZZ robots (Team 19831 Gamma, 2026-27).
 *
 * What it has, using the hardware names from that team's own code (its Drivetrain and Intake classes, Nov 2026):
 *   - 4 mecanum drive motors: "left_front", "right_front", "left_back", "right_Back" (capital B, as in that team's code)
 *   - 3 odometry pods (unpowered wheels that only count how far they roll):
 *       "pod_left" and "pod_right" roll when the robot drives forward or back,
 *       "pod_strafe" rolls when the robot drives sideways
 *   - 2 intake servos that raise and lower the intake bar: "left_intake", "right_intake"
 *   - 1 intake motor that spins the intake wheels: "intake"
 *   - an IMU ("imu") and 4 distance sensors ("front_distance", "left_distance", "back_distance", "right_distance")
 *
 * Differences from the real robot (keep these in mind):
 *   - On the real robot the pods plug into the encoder sockets of motor ports, so the code reads each pod
 *     through a motor's name. Here each pod has its own name. Only the Pedro settings file needs to know.
 *   - Pod positions and size below are placeholders until the real robot is measured.
 *   - The physics (mass, motor strength, wheel grip) are copied from the simulator's "MecDynamic Bot",
 *     so Pedro's MecDynamic tuning numbers also fit this robot. They are not the real robot's numbers.
 *
 * The driving physics are copied from DynamicMecanumBase, which hard-codes its own motor names. Copying
 * (instead of changing that shared file) keeps this branch easy to update from the original simulator.
 */
@BotConfig(name = "Mecanum Odometry Bot", filename = "mecanum_odometry_bot")
public class MecanumOdometryBot extends VirtualBot {

    /*
     * ODOMETRY POD PLACEMENT (placeholders: measure the real robot and update these)
     * Robot coordinates here: +X = toward the robot's right side, +Y = toward the robot's front, in inches,
     * measured from the center of the robot.
     */
    private static final double POD_WHEEL_DIAMETER_INCHES = 1.26;   // 32 mm, the size of a goBILDA pod wheel
    private static final double LEFT_POD_X_INCHES = -6.5;           // left pod: 6.5 in left of center
    private static final double RIGHT_POD_X_INCHES = 6.5;           // right pod: 6.5 in right of center
    private static final double STRAFE_POD_Y_INCHES = -3.0;         // sideways pod: 3 in behind center
    // The simulator has no 2000-count pod type, so the pods count 1120 ticks per wheel turn.
    private static final MotorType POD_TYPE = MotorType.Neverest40;

    public final MotorType MOTOR_TYPE = Config.DEFAULT_DRIVE_MOTOR_TYPE;

    // Drive motors, in the order the physics math below expects: back left, front left, front right, back right
    private DcMotorExDynImpl[] motors = null;

    private DeadWheelEncoder podLeft = null;
    private DeadWheelEncoder podRight = null;
    private DeadWheelEncoder podStrafe = null;

    private ServoImpl leftIntakeServo = null;
    private ServoImpl rightIntakeServo = null;
    private DcMotorExImpl intakeMotor = null;

    private BNO055IMUImpl imu = null;
    private BNO055IMUNew imuNew = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

    // Pod geometry converted to pixels (set in initialize)
    private double podRadiusPixels;
    private double leftPodXPixels;
    private double rightPodXPixels;
    private double strafePodYPixels;

    // Values the display reads (set on the physics thread, drawn on the screen thread)
    private volatile double leftArmDown = 0;     // 0 = raised, 1 = lowered (assumed: check on the real robot)
    private volatile double rightArmDown = 0;
    private volatile double intakePower = 0;

    // Shapes from mecanum_odometry_bot.fxml that change while the robot runs
    @FXML private Rectangle leftIntakeArm;
    @FXML private Rectangle rightIntakeArm;
    @FXML private Rectangle intakeRoller;

    /*
     * Drive physics, copied from DynamicMecanumBase (same values as the MecDynamic Bot)
     */
    protected double gearRatioWheel = 1.0;
    private final double wheelRadiusMeters = 2.0 / VirtualField.INCHES_PER_METER;
    private final double wheelInertia = 0.236 * (9.0/16.0) * wheelRadiusMeters * wheelRadiusMeters;
    private final double wheelCircumferenceMeters = Math.PI * 2.0 * wheelRadiusMeters;
    private final double wlAverageMeters = 15.0 / VirtualField.INCHES_PER_METER;
    private MatrixF motorTorqueToRobotForces;

    public MecanumOdometryBot() {
        super();
    }

    public void initialize() {
        super.initialize();
        hardwareMap.setActive(true);

        motors = new DcMotorExDynImpl[]{
                (DcMotorExDynImpl) hardwareMap.get(DcMotorEx.class, "left_back"),
                (DcMotorExDynImpl) hardwareMap.get(DcMotorEx.class, "left_front"),
                (DcMotorExDynImpl) hardwareMap.get(DcMotorEx.class, "right_front"),
                (DcMotorExDynImpl) hardwareMap.get(DcMotorEx.class, "right_Back")
        };
        podLeft = hardwareMap.get(DeadWheelEncoder.class, "pod_left");
        podRight = hardwareMap.get(DeadWheelEncoder.class, "pod_right");
        podStrafe = hardwareMap.get(DeadWheelEncoder.class, "pod_strafe");

        leftIntakeServo = (ServoImpl) hardwareMap.servo.get("left_intake");
        rightIntakeServo = (ServoImpl) hardwareMap.servo.get("right_intake");
        intakeMotor = (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "intake");

        distanceSensors = new VirtualRobotController.DistanceSensorImpl[]{
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "front_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "left_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "back_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "right_distance")
        };
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        imuNew = hardwareMap.get(BNO055IMUNew.class, "imu");

        hardwareMap.setActive(false);

        // The robot is drawn 18 in wide, so botWidth pixels = 18 inches
        double pixelsPerInch = botWidth / 18.0;
        podRadiusPixels = 0.5 * POD_WHEEL_DIAMETER_INCHES * pixelsPerInch;
        leftPodXPixels = LEFT_POD_X_INCHES * pixelsPerInch;
        rightPodXPixels = RIGHT_POD_X_INCHES * pixelsPerInch;
        strafePodYPixels = STRAFE_POD_Y_INCHES * pixelsPerInch;

        // Physics setup, copied from DynamicMecanumBase
        GeneralMatrixF K = new GeneralMatrixF(4, 4, new float[]{
                1, -1, (float)wlAverageMeters, 1,
                -1, -1, (float)wlAverageMeters, -1,
                -1, 1, (float)wlAverageMeters, 1,
                1, 1, (float)wlAverageMeters, -1
        });
        MatrixF M = GeneralMatrixF.diagonalMatrix(new VectorF(
                (float)chassisBody.getMass().getMass(), (float)chassisBody.getMass().getMass(),
                (float)chassisBody.getMass().getInertia(), (float)chassisBody.getMass().getMass()
        ));
        MatrixF temp1 = K.multiplied(M.inverted().multiplied(K.transposed()))
                .multiplied((float)(wheelInertia / wheelRadiusMeters));
        MatrixF temp2 = GeneralMatrixF.identityMatrix(4)
                .multiplied((float)wheelRadiusMeters);
        motorTorqueToRobotForces = K.transposed().multiplied((temp1.added(temp2)).inverted());
    }

    /**
     * Everything the robot's code can ask for by name. A name the code asks for that isn't here
     * stops the program at INIT, the same as on the real robot.
     */
    protected void createHardwareMap() {
        hardwareMap = new HardwareMap();

        // Drive motors on "hub" 0, ports 0-3
        String[] motorNames = new String[]{"left_back", "left_front", "right_front", "right_Back"};
        for (int i = 0; i < 4; i++) {
            hardwareMap.put(motorNames[i], new DcMotorExDynImpl(MOTOR_TYPE, motorController0, i));
        }

        // Intake motor on "hub" 1, port 0. The 3 pods use hub 1, ports 1-3.
        hardwareMap.put("intake", new DcMotorExImpl(MotorType.Gobilda192, motorController1, 0));
        hardwareMap.put("pod_left", new DeadWheelEncoder(POD_TYPE, motorController1, 1));
        hardwareMap.put("pod_right", new DeadWheelEncoder(POD_TYPE, motorController1, 2));
        hardwareMap.put("pod_strafe", new DeadWheelEncoder(POD_TYPE, motorController1, 3));

        hardwareMap.put("left_intake", new ServoImpl());
        hardwareMap.put("right_intake", new ServoImpl());

        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name : distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));
        hardwareMap.put("imu", new BNO055IMUNew(this, 10));
    }

    public synchronized void updateStateAndSensors(double millis) {

        // Remember where the robot was, so the pods can be turned by how far it moved
        double xOld = x;
        double yOld = y;
        double headingOld = headingRadians;

        /*
         * Drive physics, copied from DynamicMecanumBase.updateStateAndSensors
         */
        double xMeters = chassisBody.getTransform().getTranslationX();
        double yMeters = chassisBody.getTransform().getTranslationY();
        x = xMeters * VirtualField.PIXELS_PER_METER;
        y = yMeters * VirtualField.PIXELS_PER_METER;
        headingRadians = chassisBody.getTransform().getRotationAngle();

        Vector2 velocityMetersPerSec = chassisBody.getLinearVelocity();
        double angularVelocityRadiansPerSec = chassisBody.getAngularVelocity();
        double sinHd = Math.sin(headingRadians);
        double cosHd = Math.cos(headingRadians);
        double vxRobot = velocityMetersPerSec.x * cosHd + velocityMetersPerSec.y * sinHd;
        double vyRobot = -velocityMetersPerSec.x * sinHd + velocityMetersPerSec.y * cosHd;

        double[] wSpd = new double[4];
        wSpd[0] = vxRobot - vyRobot + angularVelocityRadiansPerSec * wlAverageMeters;
        wSpd[1] = -vxRobot - vyRobot + angularVelocityRadiansPerSec * wlAverageMeters;
        wSpd[2] = -vxRobot + vyRobot + angularVelocityRadiansPerSec * wlAverageMeters;
        wSpd[3] = vxRobot + vyRobot + angularVelocityRadiansPerSec * wlAverageMeters;
        for (int i = 0; i < 4; i++) {
            wSpd[i] *= 60.0 / (wheelCircumferenceMeters * MOTOR_TYPE.MAX_RPM);
        }

        float[] tauArray = new float[4];
        for (int i = 0; i < 4; i++) {
            tauArray[i] = (float) motors[i].update(millis, wSpd[i]);
        }
        VectorF forcesRobot = motorTorqueToRobotForces.multiplied(new VectorF(tauArray));
        double fx = forcesRobot.get(0) * cosHd - forcesRobot.get(1) * sinHd;
        double fy = forcesRobot.get(0) * sinHd + forcesRobot.get(1) * cosHd;
        chassisBody.applyForce(new Vector2(fx, fy));
        chassisBody.applyTorque(forcesRobot.get(2));

        imu.updateHeadingRadians(headingRadians);
        imuNew.updateHeadingRadians(headingRadians);

        final double piOver2 = Math.PI / 2.0;
        for (int i = 0; i < 4; i++) {
            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * piOver2);
            distanceSensors[i].updateDistance(x - halfBotWidth * Math.sin(sensorHeading),
                    y + halfBotWidth * Math.cos(sensorHeading), sensorHeading);
        }

        /*
         * ODOMETRY PODS (same method as the simulator's Kiwi Bot)
         * Turn the robot's movement since the last update into how far each pod wheel rolled.
         */
        double deltaX = x - xOld;
        double deltaY = y - yOld;
        double headingChange = AngleUtils.normalizeRadians(headingRadians - headingOld);
        double avgHeading = AngleUtils.normalizeRadians(headingOld + 0.5 * headingChange);
        double sin = Math.sin(avgHeading);
        double cos = Math.cos(avgHeading);
        double rightMove = deltaX * cos + deltaY * sin;     // movement toward the robot's right side
        double forwardMove = -deltaX * sin + deltaY * cos;   // movement toward the robot's front

        // Right pod counts up when the robot drives forward.
        // Left pod is mounted facing the other way (like most real robots), so it counts DOWN going forward.
        // Sideways pod counts up when the robot moves to its left.
        podRight.update((forwardMove + rightPodXPixels * headingChange) / podRadiusPixels, millis);
        podLeft.update(-(forwardMove + leftPodXPixels * headingChange) / podRadiusPixels, millis);
        podStrafe.update(-(rightMove - strafePodYPixels * headingChange) / podRadiusPixels, millis);

        /*
         * INTAKE: the motor, and the two servos.
         * The right servo is mounted mirrored, so its "lowered" is position 0. That team's code reverses it,
         * so if both arms look the same on screen, the code got the direction right.
         */
        intakeMotor.update(millis);
        intakePower = intakeMotor.getPower();
        leftArmDown = leftIntakeServo.getInternalPosition();
        rightArmDown = 1.0 - rightIntakeServo.getInternalPosition();
    }

    /**
     * Draw the robot. Runs on the screen thread.
     */
    public synchronized void updateDisplay() {
        super.updateDisplay();

        // Intake arms reach up to 12 units (about 3 in) out the front when lowered
        double leftReach = 12.0 * leftArmDown;
        double rightReach = 12.0 * rightArmDown;
        leftIntakeArm.setY(-leftReach);
        leftIntakeArm.setHeight(6 + leftReach);
        rightIntakeArm.setY(-rightReach);
        rightIntakeArm.setHeight(6 + rightReach);

        // Intake roller: green = pulling in, red = pushing out, grey = stopped
        if (intakePower > 0.05) intakeRoller.setFill(Color.LIMEGREEN);
        else if (intakePower < -0.05) intakeRoller.setFill(Color.RED);
        else intakeRoller.setFill(Color.GREY);
    }

    public void powerDownAndReset() {
        for (int i = 0; i < 4; i++) motors[i].stopAndReset();
        podLeft.stopAndReset();
        podRight.stopAndReset();
        podStrafe.stopAndReset();
        intakeMotor.stopAndReset();
        imu.close();
        chassisBody.setAngularVelocity(0);
        chassisBody.setLinearVelocity(0, 0);
    }

    /**
     * Physics body, copied from DynamicMecanumBase (same mass as the MecDynamic Bot)
     */
    public void setUpChassisBody() {
        chassisBody = new Body();
        chassisBody.setUserData(this);
        double botWidthMeters = botWidth / VirtualField.PIXELS_PER_METER;
        chassisFixture = chassisBody.addFixture(
                new org.dyn4j.geometry.Rectangle(botWidthMeters, botWidthMeters), 47.84, 0, 0);
        chassisRectangle = (org.dyn4j.geometry.Rectangle) chassisFixture.getShape();
        chassisFixture.setFilter(Filters.CHASSIS_FILTER);
        chassisBody.setMass(MassType.NORMAL);
        chassisBody.setLinearDamping(1);
        chassisBody.setAngularDamping(1);
        world.addBody(chassisBody);
    }
}
