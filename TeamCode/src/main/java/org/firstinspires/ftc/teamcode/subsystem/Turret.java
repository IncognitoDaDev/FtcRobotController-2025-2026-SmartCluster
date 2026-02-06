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
import com.smartcluster.oracleftc.hardware.OmegaPowerCollector;
import com.smartcluster.oracleftc.hardware.subsystem.OmegaActuator;
import com.smartcluster.oracleftc.hardware.subsystem.OmegaServoActuator;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.hardware.subsystem.SubsystemFlavor;
import com.smartcluster.oracleftc.hardware.wrappers.Encoder;
import com.smartcluster.oracleftc.hardware.wrappers.OmegaDcMotorImplEx;
import com.smartcluster.oracleftc.hardware.wrappers.OmegaServoImplEx;
import com.smartcluster.oracleftc.hardware.wrappers.RawEncoder;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.control.MotorFeedforward;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;
import com.smartcluster.oracleftc.math.filters.LowPassFilter;

import org.firstinspires.ftc.teamcode.roadrunner.Localizer;

@Config
public class Turret extends Subsystem {


    public final OmegaDcMotorImplEx turretUp, turretDown,turretRot;
    public final OmegaServoImplEx rightHood, leftHood;
    private OmegaPowerCollector powerCollector;
    public static TrapezoidalMotionProfile hoodMotionProfile = new TrapezoidalMotionProfile(12, 16, 16);
    public static TrapezoidalMotionProfile turretMotionProfile = new TrapezoidalMotionProfile(80, 100, 100);
    public static PIDController turretPID = new PIDController(0.025, 0.00002, 0.0019, 1);
    public static MotorFeedforward turretFeedForward = new MotorFeedforward(0.01, 0.0015, 0);

    public final OmegaActuator turret;
    public final Encoder encoder;
    public final OmegaServoActuator hood;
    private final double m = 8.502;
    private final double n = 1300.98;
    private boolean inZone;



    public Turret(OpMode opMode, OmegaPowerCollector powerCollector) {
        super(opMode);
        this.powerCollector = powerCollector;

        turretUp = new OmegaDcMotorImplEx(hardwareMap.get(DcMotorImplEx.class, "turretUp"), powerCollector, false);
        turretDown = new OmegaDcMotorImplEx(hardwareMap.get(DcMotorImplEx.class, "turretDown"), powerCollector, false);
        turretRot = new OmegaDcMotorImplEx(hardwareMap.get(DcMotorImplEx.class, "turretRotate"), powerCollector, false);

        encoder = new RawEncoder(hardwareMap.get(DcMotorImplEx.class,"turretRotate"));

        rightHood = new OmegaServoImplEx(hardwareMap.get(ServoImplEx.class, "rightHood"), powerCollector, false);
        leftHood = new OmegaServoImplEx(hardwareMap.get(ServoImplEx.class, "leftHood"), powerCollector, false);

        leftHood.getServo().setDirection(Servo.Direction.REVERSE);
        turretUp.getDcMotor().setDirection(DcMotorSimple.Direction.REVERSE);

        hood = new OmegaServoActuator(this, "hood", hoodMotionProfile, rightHood, leftHood) {
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
                if(target<0.25)target = 0.25;
                if(target>0.9)target = 0.9;
                this.target.set(target);
                return true;
            }
        };

        turret = new OmegaActuator(this, "turret", turretPID, turretMotionProfile, turretFeedForward, 2, turretRot) {
            @Override
            public Command reset() {
                return new InstantCommand(encoder::reset);
            }

            @Override
            public boolean setTarget(double target) {
                this.target.set(target);
                return true;
            }

            @Override
            public DualNum<Time> getPosition() {
                return encoder.getCurrentPosition().div(28).times(48).div(260).times(103.8);
            }


        };
    }
    public static MotorFeedforward flywheelFeedforward = new MotorFeedforward(0.1, 0.00024, 0);
    public static PIDController flywheelPID = new PIDController(0.0012, 0, 0, 0.5);
    public static LowPassFilter velocityFilter = new LowPassFilter(0.5);
    private double targetVelocity = 0; // RPM

    private final double Tolerance = 100;
    //angle between -180 and 180 to corespond with the robot heading values

    public Command trackCorner(Localizer localizer, Pose2d corner){
        return Command.builder()
                .init(()->inZone = true)
               .update(()->{
                    localizer.update();
                    Pose2d robotPose = localizer.getPose();
                   double currentX = robotPose.position.x;
                   double currentY = robotPose.position.y;

                    // Calculate vector from robot to corner
                    double dx = corner.position.x - robotPose.position.x;
                    double dy = corner.position.y - robotPose.position.y;
                   double distance = (Math.sqrt(Math.pow((corner.position.x - currentX), 2)
                           + Math.pow((corner.position.y - currentY), 2)))*2.54;
                   double velocity = m * distance + n;
                   double currentVelocity = getCurrentVelocity(); //RPM
                   double power = flywheelPID.update(velocity, currentVelocity) + flywheelFeedforward.update(velocity, 0);
                   turretUp.setPower(power);
                   turretDown.setPower(power);

                    // Angle to the corner in world space (degrees)
                    double worldAngle = Math.toDegrees(Math.atan2(dy, dx));
                    // Robot heading in degrees
                    double robotAngle = Math.toDegrees(robotPose.heading.log());
                    // Target turret angle relative to robot
                    double targetAngle =180-(worldAngle - robotAngle);

                    // Normalize the angle to -180 to 180 (shortest path)
                    while (targetAngle > 180) targetAngle -= 180;
                    while (targetAngle <= -180) targetAngle += 180;

                    turret.setTarget(targetAngle);
                   if(currentY>=Math.abs(currentX)+9*1.41 || (currentY>-46+9*1.41 && Math.abs(currentX)<23+9*1.41))inZone = false;

               })
                .finished(()->!inZone)
                .requires(this)
                .build();
    }
    public double getCurrentVelocity() {
        return velocityFilter.update((turretUp.getDcMotor().getVelocity() / 28) * 60);
    }

    public void setTargetVelocity(double velocity) {
        if (velocity < 0) velocity = 0;
        if (velocity > 6000) velocity = 6000;
        targetVelocity = velocity;
    }
    public Command setVelocityByDistance(Localizer localizer, Pose2d corner){
        return Command.builder()
                .init(()-> {
                    inZone = true;
                })
                .update(()-> {
                    localizer.update();
                    Pose2d currentPose = localizer.getPose();
                    double currentX = currentPose.position.x;
                    double currentY = currentPose.position.y;
                    double distance = (Math.sqrt(Math.pow((corner.position.x - currentX), 2)
                                    + Math.pow((corner.position.y - currentY), 2)))*2.54;
                    double velocity = m * distance + n;
                    double currentVelocity = getCurrentVelocity(); //RPM
                    double power = flywheelPID.update(velocity, currentVelocity) + flywheelFeedforward.update(velocity, 0);
                    turretUp.setPower(power);
                    turretDown.setPower(power);
                    if(currentY>=Math.abs(currentX)+9*1.41 || (currentY>-46+9*1.41 && Math.abs(currentX)<23+9*1.41))inZone = false;
                })
                .finished(()->!inZone)
                .build();
    }

    public Command update() {
        return new ParallelCommand(
                hood.update(),
//                Experimental
//                turret.update(),
                Command.builder()
                        .update(() -> {
                            double currentVelocity = getCurrentVelocity(); //RPM
                            double power = flywheelPID.update(targetVelocity, currentVelocity) + flywheelFeedforward.update(targetVelocity, 0);
                            turretUp.setPower(power);
                            turretDown.setPower(power);
                        })
                        .requires(this)
                        .build()
        );
    }

    public Command WaitForRPM(double maxMinisecond)
    {
        ElapsedTime timer = new ElapsedTime();
        return Command.builder()
                .init(timer::reset)
                .update(() ->
                {
                    telemetry.addData("VelocityErrDist", Math.abs(targetVelocity-getCurrentVelocity()));
                })
                .finished(() -> Math.abs(targetVelocity-getCurrentVelocity())<Tolerance || timer.milliseconds() > maxMinisecond)
                .build();
    }

    public Command reset()
    {
        return new SequentialCommand(
                hood.reset(),
                turret.reset()
                );
    }

    @Override
    public SubsystemFlavor flavor() {
        return SubsystemFlavor.ControlHubOnly;
    }
}
