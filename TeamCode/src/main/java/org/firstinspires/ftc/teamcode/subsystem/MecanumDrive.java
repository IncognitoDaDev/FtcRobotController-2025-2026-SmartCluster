package org.firstinspires.ftc.teamcode.subsystem;


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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.math.Pose2d;
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
import java.util.concurrent.atomic.AtomicReference;

@Config
public class MecanumDrive  {

    public DcMotorEx frontRightMotor, backRightMotor, frontLeftMotor, backLeftMotor;
    public LynxVoltageSensor voltageSensor;
    public SmartLocalizer localizer;

    public Command update()
    {
        return Command.builder()
                .update(()->{
                    lastTwist =localizer.update();
                    FtcDashboard.getInstance().sendTelemetryPacket(drawRobot(getPose()));

                }).build();
    }

    public void updateManual()
    {
        lastTwist =localizer.update();
    }

    public TelemetryPacket drawRobot(com.acmerobotics.roadrunner.Pose2dDual<Time> pose) {
        final double ROBOT_RADIUS = 9;
        TelemetryPacket packet = new TelemetryPacket();
        Canvas c = packet.fieldOverlay();


        com.acmerobotics.roadrunner.Pose2d t = pose.value();
        com.acmerobotics.roadrunner.Vector2d p = t.position;
        c.setStrokeWidth(1);
        c.strokeCircle(p.x, p.y, ROBOT_RADIUS);

        com.acmerobotics.roadrunner.Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        com.acmerobotics.roadrunner.Vector2d p1 = p.plus(halfv);
        com.acmerobotics.roadrunner.Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
        c.setStroke("#0000FF");
        c.strokeLine(p.x, p.y, p.x+pose.velocity().value().linearVel.x, p.y);
        c.strokeLine(p.x, p.y, p.x, p.y+pose.velocity().value().linearVel.y);
        packet.put("x", t.position.x);
        packet.put("y", t.position.y);
        packet.put("heading", Math.toDegrees(t.heading.log()));
        packet.put("velocityX", pose.velocity().value().linearVel.x);
        packet.put("velocityY", pose.velocity().value().linearVel.y);
        packet.put("headingVelocity", Math.toDegrees(pose.heading.velocity().get(0)));
        return packet;
    }


    public com.acmerobotics.roadrunner.Pose2dDual<Time> getPose()
    {
        Pose2dDual<Time> localizerPose = localizer.getPose();
        return new com.acmerobotics.roadrunner.Pose2dDual<>(
                new com.acmerobotics.roadrunner.Vector2dDual<>(
                        new DualNum<>(
                                new double[] {localizerPose.position.x.get(0), localizerPose.position.x.get(1)}
                        ),
                        new DualNum<>(
                                new double[] {localizerPose.position.y.get(0), localizerPose.position.y.get(1)}
                        )
                ),
                com.acmerobotics.roadrunner.Rotation2dDual.exp(new DualNum<>(
                        new double[] {localizer.getPose().heading.value().log(), localizerPose.heading.velocity().get(0)}
                ))
        );
    }

    public Command drive(ProcessedGamepad gamepad)
    {
        return new Command.CommandBuilder()
                .update(()->{
                    ProcessedGamepad.Joystick.JoystickData leftStick = gamepad.left_stick.get();
                    ProcessedGamepad.Joystick.JoystickData rightStick = gamepad.right_stick.get();

                    double boost = (gamepad.right_bumper.get() ? 1 : 0.4);

                    double rx = rightStick.x * 1.15 * boost;
                    double y = -leftStick.y * boost;
                    double x = leftStick.x * boost;

                    // Denominator is the largest motor power (absolute value) or 1
                    // This ensures all the powers maintain the same ratio,
                    // but only if at least one is out of the range [-1, 1]

                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower = (y + x + rx) / denominator;
                    double backLeftPower = (y - x + rx) / denominator;
                    double frontRightPower = (y - x - rx) / denominator;
                    double backRightPower = (y + x - rx) / denominator;

                    frontRightMotor.setPower(frontRightPower);
                    backRightMotor.setPower(backRightPower);
                    frontLeftMotor.setPower(frontLeftPower);
                    backLeftMotor.setPower(backLeftPower);
                })
                .build();
    }
    public static PIDController rotationPID = new PIDController(3.5,0, 0.09);
    public Command driveFieldCentric(ProcessedGamepad gamepad, boolean flipRed, com.acmerobotics.roadrunner.Pose2d corner)
    {

        AtomicBoolean lockedMode=new AtomicBoolean();
        lockedMode.set(lockedIn);
        return new Command.CommandBuilder()
                .update(()->{
                    ProcessedGamepad.Joystick.JoystickData leftStick = gamepad.left_stick.get();
                    ProcessedGamepad.Joystick.JoystickData rightStick = gamepad.right_stick.get();
                    if (gamepad.touchpad.pressed().get()) {
                        currentPose=new com.acmerobotics.roadrunner.Pose2d(0,0, Math.toRadians(90));
                        localizer.setPose(new com.acmerobotics.roadrunner.Pose2d(0,0, Math.toRadians(90)));
                    }

                    if(gamepad.triangle.pressed().get())
                    {
                        lockedMode.set(!lockedMode.get());
                    }

                    double botHeading = localizer.getPose().heading.value().log();
                    double boost = (gamepad.right_bumper.get() ? 1 : 0.4);

                    double rx;

                    if(lockedMode.get())
                    {
                        Vector2d dir= currentPose.position.minus(corner.position);
                        dir=dir.div(dir.norm());
                        double angle = Math.atan2(dir.y, dir.x);


                        rx = rotationPID.update(0, AngleUnit.normalizeRadians(angle-botHeading));
                    }else {
                        rx = rightStick.x * boost;
                    }
                    double y,x;
                    if(!flipRed)y=-leftStick.y * boost;
                    else y=leftStick.y*boost;
                    if(!flipRed)x = leftStick.x * boost;
                    else x=-leftStick.x*boost;
                    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    rotX = rotX * 1.1;  // Counteract imperfect strafing

                    // Denominator is the largest motor power (absolute value) or 1
                    // This ensures all the powers maintain the same ratio,
                    // but only if at least one is out of the range [-1, 1]

                    // Field centric drive try
                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                    double frontLeftPower = (rotY + rotX + rx) / denominator;
                    double backLeftPower = (rotY - rotX + rx) / denominator;
                    double frontRightPower = (rotY - rotX - rx) / denominator;
                    double backRightPower = (rotY + rotX - rx) / denominator;

                    frontRightMotor.setPower(frontRightPower);
                    backRightMotor.setPower(backRightPower);
                    frontLeftMotor.setPower(frontLeftPower);
                    backLeftMotor.setPower(backLeftPower);
                })
                .build();
    }

    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // drive model parameters
        public double inPerTick = 0.00198489276065501461101615482164;
        public double lateralInPerTick = 0.0011684157603547806;
        public double trackWidthTicks = 6449.470659804927;

        // feedforward parameters (in tick units)
        public double kS = 1.5408819962455707;
        public double kV = 0.00025079813810809705;
        public double kA = 0.00008;

        // path profile parameters (in inches)
        public double maxWheelVel = 75;
        public double minProfileAccel = -75;
        public double maxProfileAccel = 75;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain =15;
        public double lateralGain = 2.5;
        public double headingGain = 3; // shared with turn

        public double axialVelGain = 0.5;
        public double lateralVelGain = 0.5;
        public double headingVelGain = 0.6; // shared with turn
    }

    public static Params PARAMS = new Params();
    public static com.acmerobotics.roadrunner.Pose2d currentPose = new com.acmerobotics.roadrunner.Pose2d(0,0,0);
    public static boolean lockedIn = false;
    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final LazyImu lazyImu;


    private final LinkedList<com.acmerobotics.roadrunner.Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);
    private final Telemetry telemetry;

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry=telemetry;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);


        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        localizer=new SmartLocalizer(hardwareMap, telemetry);
        frontRightMotor=hardwareMap.get(DcMotorEx.class, "frontRight");
        backRightMotor=hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeftMotor=hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeftMotor=hardwareMap.get(DcMotorEx.class, "backLeft");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        voltageSensor=hardwareMap.getAll(LynxVoltageSensor.class).iterator().next();

        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<com.acmerobotics.roadrunner.Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<com.acmerobotics.roadrunner.Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        frontLeftMotor.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        backLeftMotor.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        backRightMotor.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        frontRightMotor.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public Command p2p(com.acmerobotics.roadrunner.Pose2d target)
    {
        return new P2PCommand(target);
    }
    public final class P2PCommand extends Command {
        private final com.acmerobotics.roadrunner.Pose2dDual<com.acmerobotics.roadrunner.Time> target;
        public P2PCommand(com.acmerobotics.roadrunner.Pose2dDual<com.acmerobotics.roadrunner.Time> target)
        {
            this.target=target;
        }
        public P2PCommand(com.acmerobotics.roadrunner.Pose2d target)
        {
            this.target=new com.acmerobotics.roadrunner.Pose2dDual<>(
                    new DualNum<>(new double[]{target.position.x,0,0}),
                    new DualNum<>(new double[]{target.position.y,0,0}),
                    new DualNum<>(new double[]{target.heading.log(),0,0})
            );

        }
        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
        }

        @Override
        public boolean finished() {
            com.acmerobotics.roadrunner.Pose2d error = target.value().minusExp(getPose().value());
            return error.position.norm()<2 && Math.abs(error.heading.log())<Math.toRadians(5);
        }



        @Override
        public Set<Subsystem> requires() {
            return super.requires();
        }

        @Override
        public void update() {

            telemetry.addData("targetX", target.value().position.x);
            telemetry.addData("targetY", target.value().position.y);
            telemetry.addData("targetH", Math.toDegrees(target.heading.value().log()));
            com.acmerobotics.roadrunner.Pose2d error = target.value().minusExp(getPose().value());

            if (error.position.norm()<2 && Math.abs(error.heading.log())<Math.toRadians(5)) {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                frontRightMotor.setPower(0);
                return;
            }
            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<com.acmerobotics.roadrunner.Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(target, getPose().value(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<com.acmerobotics.roadrunner.Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
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


    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                com.acmerobotics.roadrunner.Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run( TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }
            com.acmerobotics.roadrunner.Pose2dDual<com.acmerobotics.roadrunner.Time> txWorldTarget = timeTrajectory.get(t);

            com.acmerobotics.roadrunner.Pose2d error = txWorldTarget.value().minusExp(getPose().value());

            if (t >= timeTrajectory.duration && error.position.norm()<2 && Math.abs(error.heading.log())<Math.toRadians(5)) {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                frontRightMotor.setPower(0);
                return false;
            }

            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<com.acmerobotics.roadrunner.Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, getPose().value(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<com.acmerobotics.roadrunner.Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
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

            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.value().log()));

            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
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
    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run( TelemetryPacket p) {
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

            com.acmerobotics.roadrunner.Pose2dDual<com.acmerobotics.roadrunner.Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<com.acmerobotics.roadrunner.Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, getPose().value(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<com.acmerobotics.roadrunner.Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            frontLeftMotor.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            backLeftMotor.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            backRightMotor.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            frontRightMotor.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

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
    private Twist2dDual<Time> lastTwist=new Twist2dDual<Time>(new Vector2dDual<Time>(new com.smartcluster.oracleftc.math.DualNum<Time>(0.0), new com.smartcluster.oracleftc.math.DualNum<Time>(0.0)), new com.smartcluster.oracleftc.math.DualNum<Time>(0.0));
    public PoseVelocity2d updatePoseEstimate() {
        localizer.update();
        Twist2dDual<Time> twist = lastTwist;
        PoseVelocity2d velocity = new PoseVelocity2d(
                new com.acmerobotics.roadrunner.Vector2d(
                        twist.line.x.get(1),
                        twist.line.y.get(1)
                ),
                twist.angle.get(1)
        );

        poseHistory.add(getPose().value());

        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(getPose().value()));

        return velocity;
    }

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

    public TrajectoryActionBuilder actionBuilder(com.acmerobotics.roadrunner.Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
}

