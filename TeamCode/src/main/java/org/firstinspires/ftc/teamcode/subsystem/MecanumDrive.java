package org.firstinspires.ftc.teamcode.subsystem;
//avem variabile false
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.math.Pose2dDual;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.Twist2dDual;
import com.smartcluster.oracleftc.math.Vector2dDual;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.utils.ProcessedGamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.roadrunner.oraclelocalizer.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.oraclelocalizer.SmartLocalizer;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class MecanumDrive {

    // ============================================================================
    // CONSTANTS & CONFIGURATION
    // ============================================================================

    private static final double ROBOT_RADIUS = 9.0;
    private static final double STRAFE_MULTIPLIER = 1.1; // Counteract imperfect strafing
    private static final int POSE_HISTORY_SIZE = 100;
    private static final double POSITION_TOLERANCE = 2.0; // inches
    private static final double HEADING_TOLERANCE = Math.toRadians(5); // radians

    public static PIDController rotationPID = new PIDController(3.5, 0, 0.09);
    public static com.acmerobotics.roadrunner.Pose2d currentPose =
            new com.acmerobotics.roadrunner.Pose2d(0, 0, 0);
    public static boolean lockedIn = false;

    // ============================================================================
    // HARDWARE
    // ============================================================================

    public DcMotorEx frontRightMotor, backRightMotor, frontLeftMotor, backLeftMotor;
    public LynxVoltageSensor voltageSensor;
    public SmartLocalizer localizer;
    public final LazyImu lazyImu;
    private final Telemetry telemetry;

    // ============================================================================
    // ROADRUNNER COMPONENTS
    // ============================================================================

    public final MecanumKinematics kinematics;
    public final TurnConstraints defaultTurnConstraints;
    public final VelConstraint defaultVelConstraint;
    public final AccelConstraint defaultAccelConstraint;

    private final LinkedList<com.acmerobotics.roadrunner.Pose2d> poseHistory = new LinkedList<>();
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    private Twist2dDual<Time> lastTwist = new Twist2dDual<>(
            new Vector2dDual<>(
                    new com.smartcluster.oracleftc.math.DualNum<>(0.0),
                    new com.smartcluster.oracleftc.math.DualNum<>(0.0)
            ),
            new com.smartcluster.oracleftc.math.DualNum<>(0.0)
    );

    // ============================================================================
    // PARAMETERS
    // ============================================================================

    @Config
    public static class Params {
        // IMU orientation
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // Drive model parameters
        public double inPerTick = 0.00198489276065501461101615482164;
        public double lateralInPerTick = 0.0012626677151045622;
        public double trackWidthTicks = 6679.434891050771;

        // Feedforward parameters (in tick units)
        public double kS = 1.781506346064614;
        public double kV = 0.0001932892381012636;
        public double kA = 0.000125;

        // Path profile parameters (in inches)
        public double maxWheelVel = 80;
        public double minProfileAccel = -80;
        public double maxProfileAccel = 80;

        // Turn profile parameters (in radians)
        public double maxAngVel = Math.PI;
        public double maxAngAccel = Math.PI;

        // Path controller gains
        public double axialGain = 12;
        public double lateralGain = 12;
        public double headingGain = 10;

        public double axialVelGain = 1.07;
        public double lateralVelGain = 0.2;
        public double headingVelGain = 0.4;
    }

    public static Params PARAMS = new Params();

    // ============================================================================
    // CONSTRUCTORS
    // ============================================================================

    public MecanumDrive(OpMode opMode) {
        this(opMode.hardwareMap, opMode.telemetry);
    }

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        // Initialize localizer
        localizer = new SmartLocalizer(hardwareMap, telemetry);

        // Initialize motors
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");

        // Set motor directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero power
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize voltage sensor
        voltageSensor = hardwareMap.getAll(LynxVoltageSensor.class).iterator().next();

        // Initialize IMU
        lazyImu = new LazyHardwareMapImu(
                hardwareMap,
                "imu",
                new RevHubOrientationOnRobot(PARAMS.logoFacingDirection, PARAMS.usbFacingDirection)
        );

        // Initialize RoadRunner components
        kinematics = new MecanumKinematics(
                PARAMS.inPerTick * PARAMS.trackWidthTicks,
                PARAMS.inPerTick / PARAMS.lateralInPerTick
        );

        defaultTurnConstraints = new TurnConstraints(
                PARAMS.maxAngVel,
                -PARAMS.maxAngAccel,
                PARAMS.maxAngAccel
        );

        defaultVelConstraint = new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                new AngularVelConstraint(PARAMS.maxAngVel)
        ));

        defaultAccelConstraint = new ProfileAccelConstraint(
                PARAMS.minProfileAccel,
                PARAMS.maxProfileAccel
        );

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    // ============================================================================
    // COMMAND METHODS
    // ============================================================================

    /**
     * Creates a command that updates the localizer and sends telemetry
     */
    public Command update() {
        return Command.builder()
                .update(() -> {
                    lastTwist = localizer.update();
                    FtcDashboard.getInstance().sendTelemetryPacket(drawRobot(getPose()));
                })
                .build();
    }

    /**
     * Robot-centric drive command
     */
    public Command drive(ProcessedGamepad gamepad) {
        return Command.builder()
                .update(() -> {
                    ProcessedGamepad.Joystick.JoystickData leftStick = gamepad.left_stick.get();
                    ProcessedGamepad.Joystick.JoystickData rightStick = gamepad.right_stick.get();

                    double boost = gamepad.right_bumper.get() ? 1.0 : 0.4;

                    double rx = rightStick.x * 1.15 * boost;
                    double y = -leftStick.y * boost;
                    double x = leftStick.x * boost;

                    setMotorPowersFromRobotCentric(x, y, rx);
                })
                .build();
    }

    /**
     * Field-centric drive command with optional heading lock
     *
     * @param gamepad Controller input
     * @param flipRed Whether to flip controls for red alliance
     * @param corner Target corner for heading lock
     */
    public Command driveFieldCentric(ProcessedGamepad gamepad, boolean flipRed,
                                     com.acmerobotics.roadrunner.Pose2d corner) {
        AtomicBoolean lockedMode = new AtomicBoolean(lockedIn);

        return Command.builder()
                .update(() -> {
                    ProcessedGamepad.Joystick.JoystickData leftStick = gamepad.left_stick.get();
                    ProcessedGamepad.Joystick.JoystickData rightStick = gamepad.right_stick.get();

                    // Reset pose on dpad left press
                    if (gamepad.dpad_left.pressed().get()) {
                        currentPose = new com.acmerobotics.roadrunner.Pose2d(0, 0, Math.toRadians(90));
                        localizer.setPose(currentPose);
                    }

                    // Toggle locked mode on triangle press
                    if (gamepad.triangle.pressed().get()) {
                        lockedMode.set(!lockedMode.get());
                    }

                    double botHeading = localizer.getPose().heading.log().get(0);
                    double boost = gamepad.right_bumper.get() ? 1.0 : 0.4;

                    // Calculate rotation based on mode
                    double rx;
                    if (lockedMode.get()) {
                        Vector2d dir = currentPose.position.minus(corner.position);
                        dir = dir.div(dir.norm());
                        double targetAngle = Math.atan2(dir.y, dir.x);
                        rx = rotationPID.update(0, AngleUnit.normalizeRadians(targetAngle - botHeading));
                    } else {
                        rx = rightStick.x * boost;
                    }

                    // Handle alliance flipping
                    double y = flipRed ? leftStick.y * boost : -leftStick.y * boost;
                    double x = flipRed ? -leftStick.x * boost : leftStick.x * boost;

                    // Convert to field-centric
                    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    rotX *= STRAFE_MULTIPLIER;

                    setMotorPowersFromRobotCentric(rotX, rotY, rx);
                })
                .build();
    }

    /**
     * Point-to-point navigation command
     */
    public Command p2p(com.acmerobotics.roadrunner.Pose2d target) {
        return new P2PCommand(target);
    }

    // ============================================================================
    // UTILITY METHODS
    // ============================================================================

    /**
     * Get the current pose as a RoadRunner Pose2dDual
     */
    public com.acmerobotics.roadrunner.Pose2dDual<Time> getPose() {
        Pose2dDual<Time> localizerPose = localizer.getPose();
        return new com.acmerobotics.roadrunner.Pose2dDual<>(
                new com.acmerobotics.roadrunner.Vector2dDual<>(
                        new DualNum<>(new double[]{
                                localizerPose.position.x.get(0),
                                localizerPose.position.x.get(1)
                        }),
                        new DualNum<>(new double[]{
                                localizerPose.position.y.get(0),
                                localizerPose.position.y.get(1)
                        })
                ),
                com.acmerobotics.roadrunner.Rotation2dDual.exp(new DualNum<>(new double[]{
                        localizerPose.heading.value().log(),
                        localizerPose.heading.velocity().get(0)
                }))
        );
    }

    /**
     * Set motor powers from robot-centric inputs
     */
    private void setMotorPowersFromRobotCentric(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Set drive powers using PoseVelocity2d
     */
    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<com.acmerobotics.roadrunner.Time> wheelVels =
                new MecanumKinematics(1).inverse(PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1.0;
        for (DualNum<com.acmerobotics.roadrunner.Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        frontLeftMotor.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        backLeftMotor.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        backRightMotor.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        frontRightMotor.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    /**
     * Update pose estimate and return velocity
     */
    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = lastTwist;
        PoseVelocity2d velocity = new PoseVelocity2d(
                new com.acmerobotics.roadrunner.Vector2d(
                        twist.line.x.get(1),
                        twist.line.y.get(1)
                ),
                twist.angle.get(1)
        );

        poseHistory.add(getPose().value());
        while (poseHistory.size() > POSE_HISTORY_SIZE) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(getPose().value()));

        return velocity;
    }

    /**
     * Draw robot on dashboard
     */
    public TelemetryPacket drawRobot(com.acmerobotics.roadrunner.Pose2dDual<Time> pose) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas c = packet.fieldOverlay();

        com.acmerobotics.roadrunner.Pose2d t = pose.value();
        com.acmerobotics.roadrunner.Vector2d p = t.position;

        // Draw robot circle
        c.setStrokeWidth(1);
        c.strokeCircle(p.x, p.y, ROBOT_RADIUS);

        // Draw heading indicator
        com.acmerobotics.roadrunner.Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        com.acmerobotics.roadrunner.Vector2d p1 = p.plus(halfv);
        com.acmerobotics.roadrunner.Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);

        // Draw velocity vectors
        c.setStroke("#0000FF");
        c.strokeLine(p.x, p.y, p.x + pose.velocity().value().linearVel.x, p.y);
        c.strokeLine(p.x, p.y, p.x, p.y + pose.velocity().value().linearVel.y);

        // Add telemetry data
        packet.put("x", t.position.x);
        packet.put("y", t.position.y);
        packet.put("heading", Math.toDegrees(t.heading.log()));
        packet.put("velocityX", pose.velocity().value().linearVel.x);
        packet.put("velocityY", pose.velocity().value().linearVel.y);
        packet.put("headingVelocity", Math.toDegrees(pose.heading.velocity().get(0)));

        return packet;
    }

    /**
     * Draw pose history on canvas
     */
    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (com.acmerobotics.roadrunner.Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;
            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    /**
     * Create trajectory action builder
     */
    public TrajectoryActionBuilder actionBuilder(com.acmerobotics.roadrunner.Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(0.25, 0.1, 1e-2)
                ),
                beginPose,
                0.0,
                defaultTurnConstraints,
                defaultVelConstraint,
                defaultAccelConstraint
        );
    }

    // ============================================================================
    // NESTED CLASSES
    // ============================================================================

    /**
     * Point-to-point navigation command
     */
    public final class P2PCommand extends Command {
        private final com.acmerobotics.roadrunner.Pose2dDual<com.acmerobotics.roadrunner.Time> target;

        public P2PCommand(com.acmerobotics.roadrunner.Pose2dDual<com.acmerobotics.roadrunner.Time> target) {
            this.target = target;
        }

        public P2PCommand(com.acmerobotics.roadrunner.Pose2d target) {
            this.target = new com.acmerobotics.roadrunner.Pose2dDual<>(
                    new DualNum<>(new double[]{target.position.x, 0, 0}),
                    new DualNum<>(new double[]{target.position.y, 0, 0}),
                    new DualNum<>(new double[]{target.heading.log(), 0, 0})
            );
        }

        @Override
        public boolean finished() {
            com.acmerobotics.roadrunner.Pose2d error = target.value().minusExp(getPose().value());
            return error.position.norm() < POSITION_TOLERANCE &&
                    Math.abs(error.heading.log()) < HEADING_TOLERANCE;
        }

        @Override
        public Set<Subsystem> requires() {
            return super.requires();
        }

        @Override
        public void update() {
            com.acmerobotics.roadrunner.Pose2d error = target.value().minusExp(getPose().value());

            // Stop if at target
            if (error.position.norm() < POSITION_TOLERANCE &&
                    Math.abs(error.heading.log()) < HEADING_TOLERANCE) {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                frontRightMotor.setPower(0);
                return;
            }

            // Update telemetry
            telemetry.addData("targetX", target.value().position.x);
            telemetry.addData("targetY", target.value().position.y);
            telemetry.addData("targetH", Math.toDegrees(target.value().heading.log()));

            // Calculate command
            PoseVelocity2d robotVelRobot = updatePoseEstimate();
            PoseVelocity2dDual<com.acmerobotics.roadrunner.Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            ).compute(target, getPose().value(), robotVelRobot);

            driveCommandWriter.write(new DriveCommandMessage(command));

            // Convert to wheel velocities and apply
            MecanumKinematics.WheelVelocities<com.acmerobotics.roadrunner.Time> wheelVels =
                    kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            MotorFeedforward feedforward = new MotorFeedforward(
                    PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick,
                    PARAMS.kA / PARAMS.inPerTick
            );

            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            frontLeftMotor.setPower(leftFrontPower);
            backLeftMotor.setPower(leftBackPower);
            backRightMotor.setPower(rightBackPower);
            frontRightMotor.setPower(rightFrontPower);
        }
    }

    /**
     * Follow trajectory action for autonomous
     */
    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;
        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2))
            );

            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                com.acmerobotics.roadrunner.Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            com.acmerobotics.roadrunner.Pose2dDual<com.acmerobotics.roadrunner.Time> txWorldTarget =
                    timeTrajectory.get(t);
            com.acmerobotics.roadrunner.Pose2d error = txWorldTarget.value().minusExp(getPose().value());

            // Check if finished
            if (t >= timeTrajectory.duration &&
                    error.position.norm() < POSITION_TOLERANCE &&
                    Math.abs(error.heading.log()) < HEADING_TOLERANCE) {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                frontRightMotor.setPower(0);
                return false;
            }

            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            // Calculate and apply control
            PoseVelocity2d robotVelRobot = updatePoseEstimate();
            PoseVelocity2dDual<com.acmerobotics.roadrunner.Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            ).compute(txWorldTarget, getPose().value(), robotVelRobot);

            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<com.acmerobotics.roadrunner.Time> wheelVels =
                    kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            MotorFeedforward feedforward = new MotorFeedforward(
                    PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick,
                    PARAMS.kA / PARAMS.inPerTick
            );

            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            frontLeftMotor.setPower(leftFrontPower);
            backLeftMotor.setPower(leftBackPower);
            backRightMotor.setPower(rightBackPower);
            frontRightMotor.setPower(rightFrontPower);

            // Update telemetry
            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().value().heading.toDouble()));
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // Draw on dashboard
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, getPose().value());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            FtcDashboard.getInstance().sendTelemetryPacket(p);
            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    /**
     * Turn action for autonomous
     */
    public final class TurnAction implements Action {
        private final TimeTurn turn;
        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                frontRightMotor.setPower(0);
                return false;
            }

            com.acmerobotics.roadrunner.Pose2dDual<com.acmerobotics.roadrunner.Time> txWorldTarget =
                    turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();
            PoseVelocity2dDual<com.acmerobotics.roadrunner.Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            ).compute(txWorldTarget, getPose().value(), robotVelRobot);

            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<com.acmerobotics.roadrunner.Time> wheelVels =
                    kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            MotorFeedforward feedforward = new MotorFeedforward(
                    PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick,
                    PARAMS.kA / PARAMS.inPerTick
            );

            frontLeftMotor.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            backLeftMotor.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            backRightMotor.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            frontRightMotor.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage,
                    feedforward.compute(wheelVels.leftFront) / voltage,
                    feedforward.compute(wheelVels.leftBack) / voltage,
                    feedforward.compute(wheelVels.rightBack) / voltage,
                    feedforward.compute(wheelVels.rightFront) / voltage
            ));

            // Draw on dashboard
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, getPose().value());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }
}