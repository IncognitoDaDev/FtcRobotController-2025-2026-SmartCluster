package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.hardware.subsystem.ServoActuator;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.hardware.subsystem.SubsystemFlavor;
import com.smartcluster.oracleftc.math.control.MotorFeedforward;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;
import com.smartcluster.oracleftc.math.filters.LowPassFilter;

@Config
public class Turret extends Subsystem {


    private final DcMotorImplEx turretUp, turretDown;
    private final ServoImplEx rightHood, leftHood;
    private final OracleLynxVoltageSensor voltageSensor;
    public static TrapezoidalMotionProfile hoodMotionProfile = new TrapezoidalMotionProfile(12, 16, 16);
    public final ServoActuator hood;
    private final double m = 8.502;
    private final double x = 1.5;


    public Turret(OpMode opMode) {
        super(opMode);
        voltageSensor = hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();
        turretUp = hardwareMap.get(DcMotorImplEx.class, "turretUp");
        turretUp.setDirection(DcMotorSimple.Direction.REVERSE);
        turretDown = hardwareMap.get(DcMotorImplEx.class, "turretDown");
        hardwareMap.get(ServoImplEx.class, "leftHood");
        rightHood = hardwareMap.get(ServoImplEx.class, "rightHood");
        leftHood = hardwareMap.get(ServoImplEx.class, "leftHood");
        hood = new ServoActuator(this, "hood", hoodMotionProfile, rightHood, leftHood) {
            @Override
            public Command reset() {
                setTarget(0.4);
                return new InstantCommand(() -> {
                    rightHood.setPosition(this.target.get());
                    leftHood.setPosition(this.target.get());
                });
            }

            @Override
            public boolean setTarget(double target) {
                this.target.set(target);
                return true;
            }
        };
    }

    public static MotorFeedforward flywheelFeedforward = new MotorFeedforward(0.1, 0.00019, 0);
    public static PIDController flywheelPID = new PIDController(0.0012, 0, 0, 0.5);
    public static LowPassFilter velocityFilter = new LowPassFilter(0.5);
    private double targetVelocity = 0; // RPM

    public double getCurrentVelocity() {
        return velocityFilter.update((turretUp.getVelocity() / 28) * 60);
    }

    public void setTargetVelocity(double velocity) {
        if (velocity < 0) velocity = 0;
        if (velocity > 6000) velocity = 6000;
        targetVelocity = velocity;
    }
    public void setVelocityByDistance(Pose2d currentPose, Pose2d corner){
        double distance = Math.sqrt(Math.pow((corner.position.x-currentPose.position.x),2)+Math.pow((corner.position.y-currentPose.position.y),2));
        double velocity = m*distance+x;
        setTargetVelocity(velocity);
    }

    public Command update() {
        return new ParallelCommand(
                hood.update(),
                Command.builder()
                        .update(() -> {
                            double currentVelocity = getCurrentVelocity(); //RPM
                            double power = flywheelPID.update(targetVelocity, currentVelocity) + flywheelFeedforward.update(targetVelocity, 0);
                            power = power * (Robot.nominalVoltage / voltageSensor.getVoltage());
                            turretUp.setPower(power);
                            turretDown.setPower(power);
                        })
                        .requires(this)
                        .build()
        );
    }

    public Command reset()
    {
        return hood.reset();
    }

    @Override
    public SubsystemFlavor flavor() {
        return SubsystemFlavor.ControlHubOnly;
    }
}
