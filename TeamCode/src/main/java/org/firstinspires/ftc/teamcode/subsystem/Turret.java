package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.hardware.subsystem.Actuator;
import com.smartcluster.oracleftc.hardware.subsystem.ServoActuator;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.hardware.subsystem.SubsystemFlavor;
import com.smartcluster.oracleftc.hardware.wrappers.Encoder;
import com.smartcluster.oracleftc.hardware.wrappers.RawEncoder;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.control.MotorFeedforward;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;
import com.smartcluster.oracleftc.math.filters.LowPassFilter;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

@Config
public class Turret extends Subsystem {


    private final DcMotorImplEx turretUp, turretDown, turretRot;
    private final ServoImplEx rightHood, leftHood;
    private final OracleLynxVoltageSensor voltageSensor;
    public static TrapezoidalMotionProfile hoodMotionProfile = new TrapezoidalMotionProfile(12, 16, 16);
    public static TrapezoidalMotionProfile turretMotionProfile = new TrapezoidalMotionProfile(80, 100, 100);
    public static PIDController turretPID = new PIDController(0.025, 0.00002, 0.0019, 1);
    public static MotorFeedforward turretFeedForward = new MotorFeedforward(0.01, 0.0015, 0);

    public final Actuator turret;
    public final Encoder encoderRot;
    public final ServoActuator hood;

    public MecanumDrive drive;
    private Pose2d goal;

    private AtomicBoolean enabledVel = new AtomicBoolean(true);
    public AtomicBoolean isAboutToShot = new AtomicBoolean(false);

    // Velocity linear regression constants
// Formula: velocity = VELOCITY_SLOPE * distance_cm + VELOCITY_INTERCEPT
    private static final double VELOCITY_SLOPE = 5.202340;
    private static final double VELOCITY_INTERCEPT = 1689.019787;

    // Hood angle linear regression constants
// Formula: hood = HOOD_SLOPE * velocity + HOOD_INTERCEPT
    private static final double HOOD_SLOPE = 0.000271;
    private static final double HOOD_INTERCEPT = -0.277722;


    public Turret(OpMode opMode) {
        super(opMode);

        voltageSensor = hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();

        turretUp = hardwareMap.get(DcMotorImplEx.class, "turretUp");
        turretUp.setDirection(DcMotorSimple.Direction.REVERSE);

//        encoderVel = new RawEncoder(hardwareMap.get(DcMotorImplEx.class, "turretUp"));
        turretDown = hardwareMap.get(DcMotorImplEx.class, "turretDown");


        turretRot = hardwareMap.get(DcMotorImplEx.class, "turretRotate");
        encoderRot = new RawEncoder(hardwareMap.get(DcMotorImplEx.class,"turretRotate"));

        hardwareMap.get(ServoImplEx.class, "leftHood");
        rightHood = hardwareMap.get(ServoImplEx.class, "rightHood");
        leftHood = hardwareMap.get(ServoImplEx.class, "leftHood");

        leftHood.setDirection(Servo.Direction.REVERSE);
        hood = new ServoActuator(this, "hood", hoodMotionProfile, rightHood, leftHood) {
            @Override
            public Command reset() {

                return new InstantCommand(() -> {
                    setTarget(0.4);
                    rightHood.setPosition(this.target.get());
                    leftHood.setPosition(this.target.get());
                });
            }

            @Override
            public boolean setTarget(double target) {
                if(target<0.25) target = 0.25;
                if(target>0.9) target = 0.9;
                this.target.set(target);
                return true;
            }
        };

        turret = new Actuator(this, "turret", turretPID, turretMotionProfile, turretFeedForward, 2, turretRot) {
            @Override
            public Command reset() {
                return new InstantCommand(encoderRot::reset);
            }

            @Override
            public boolean setTarget(double target) {
                this.target.set(target);
                return true;
            }

            @Override
            public DualNum<Time> getPosition() {
                return encoderRot.getCurrentPosition().div(28).times(48).div(260).times(103.8);
            }
        };
    }

    public void setTracking(MecanumDrive drive, Pose2d goal)
    {
        this.drive = drive;
        this.goal = goal;
    }

    public static MotorFeedforward flywheelFeedforward = new MotorFeedforward(0.1, 0.00024, 0);
    public static PIDController flywheelPID = new PIDController(0.0012, 0, 0, 0.5);
    public static LowPassFilter velocityFilter = new LowPassFilter(0.5);
    private double targetVelocity = 0; // RPM

    private final double Tolerance = 100;

    public double getCurrentVelocity() {
        return velocityFilter.update((turretDown.getVelocity() / 28) * 60);
    }

    public void setTargetVelocity(double velocity) {
        if (velocity < 0) velocity = 0;
        if (velocity > 6000) velocity = 6000;
        targetVelocity = velocity;
    }

    public double getDistanceToTarget(Pose2d currPos, com.acmerobotics.roadrunner.Pose2d corner){
        double currentX = currPos.position.x;
        double currentY = currPos.position.y;

        double dx = corner.position.x - currentX;
        double dy = corner.position.y - currentY;
        return Math.sqrt(dx * dx + dy * dy) * 2.54; // returns distance in cm
    }

    public void setVelocityAndAngleByDist(Pose2d currPos, Pose2d corner){
        double velocity = VELOCITY_SLOPE * getDistanceToTarget(currPos, corner) + VELOCITY_INTERCEPT;
        double angle = getCurrentVelocity()*HOOD_SLOPE + HOOD_INTERCEPT;

        // No longer in zone? You say so?! Stop wasting energy then!!! - R
        enabledVel.set(isInsideTheZone(currPos).get());

        setTargetVelocity(velocity);
        hood.setTarget(angle);
    }

    public Supplier<Boolean> isInsideTheZone(Pose2d pose)
    {
        double BigTriangle = Math.abs(pose.position.x)-4;
        double TinyTriangle = -Math.abs(pose.position.x)-7;
        return () -> pose.position.y >= BigTriangle || pose.position.y <= TinyTriangle;
    }

    public Command update() {
        return new ParallelCommand(
                hood.update(),
                Command.builder()
                        .update(() -> {
                            if (enabledVel.get()) {
                                double currentVelocity = getCurrentVelocity(); //RPM
                                double power = flywheelPID.update(targetVelocity, currentVelocity) + flywheelFeedforward.update(targetVelocity, 0);
                                power = power * (Robot.nominalVoltage / voltageSensor.getVoltage());

                                turretUp.setPower(power);
                                turretDown.setPower(power);
                            }
                            else
                            {
                                turretUp.setPower(0);
                                turretDown.setPower(0);
                            }
                        })
                        .requires(this)
                        .build()
        );
    }

    public Command VelocityUpdate() {
        return Command.builder()
                .update(() -> {
                    if (isAboutToShot.get()) setVelocityAndAngleByDist(drive.getPose().value(), goal);
                    else {
                        setTargetVelocity(500);
                        hood.setTarget(0.55);
                    }
                })
                .requires(this)
                .build();
    }

    public Command WaitForRPM(double maxDuration)
    {
        ElapsedTime timer = new ElapsedTime();
        return Command.builder()
                .init(timer::reset)
                .update(() ->
                {
                    telemetry.addData("VelocityErrDist", Math.abs(targetVelocity-getCurrentVelocity()));
                })
                .finished(() -> Math.abs(targetVelocity-getCurrentVelocity())<Tolerance || timer.milliseconds() > maxDuration)
                .build();
    }

//    double MIN_HOOD_VAL = 0.25, MAX_HOOD_VAL = 0.9, MAX_N_HOOD_VAL = 100, MIN_N_HOOD_VAL = 0;
//    public void setNormalizedHood(double input)
//    {
//        input = (input < MIN_N_HOOD_VAL ? MIN_N_HOOD_VAL : (Math.min(input, MAX_N_HOOD_VAL)));
//        hood.setTarget((input - MIN_N_HOOD_VAL) * (MAX_HOOD_VAL - MIN_HOOD_VAL) / (MAX_N_HOOD_VAL - MIN_N_HOOD_VAL) + MIN_HOOD_VAL);
//    }

    public Command reset()
    {
        return new SequentialCommand(
                hood.reset(),
                turret.reset()
                );
    }

    @Override
    public SubsystemFlavor flavor() {
        return SubsystemFlavor.ExpansionHubOnly;
    }
}
