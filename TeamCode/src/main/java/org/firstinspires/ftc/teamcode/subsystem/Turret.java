package org.firstinspires.ftc.teamcode.subsystem;



import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.hardware.wrappers.Encoder;
import com.smartcluster.oracleftc.hardware.wrappers.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.hardware.subsystem.Actuator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.smartcluster.oracleftc.hardware.subsystem.ServoActuator;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.math.DualNum;

import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;

import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

@Config
public class Turret extends Subsystem {

    final double HOOD_MIN_ANGLE = 0.0;   // Example: Angle when servo is at position 0.0
    final double HOOD_MAX_ANGLE = 60.0;
    public final double MOTOR_TO_TURRET_RATIO =  260.0/48;
    public final double ENCODER_TICKS_PER_ROTATION = 384.5*MOTOR_TO_TURRET_RATIO;
    public final double ENCODER_TICKS_PER_DEGREE = ENCODER_TICKS_PER_ROTATION/360;
    private final String name;
    public DcMotorEx turret1,turret2,rot;
    public ServoImplEx s1,s2;
//    public static PIDController shootPidController = new PIDController(0.0,0.0,0.0);
    public static TrapezoidalMotionProfile shootMotionProfile = new TrapezoidalMotionProfile(6000,40000,20000);
    public static PIDController rotationalPidController = new PIDController(0.0002,0.0002,0.0003);
    public static TrapezoidalMotionProfile rotationalMotionProfile = new TrapezoidalMotionProfile(6000,1000,1000);
    public final Encoder rotate;
    public double[] Speed ={0,4500,5000,6000};
    private final OracleLynxVoltageSensor voltageSensor;
    public static double tolerance = 2;

    private final ElapsedTime time = new ElapsedTime();
    public static double Kv = 1.1;
    public static double Ka = 0;
    public static double Ks = 0.001;
    FeedforwardCoefficients coefficients = new FeedforwardCoefficients(Kv,Ka,Ks);
    BasicFeedforward shootController = new BasicFeedforward(coefficients);
    public double targetSpeed = 0;
    public double targetAngle = 0;

    public GoBildaPinpointDriver pinpoint;
    public Actuator turret;
    public Actuator rotation;
    public ServoActuator hood;
    PinpointLocalizer localizer;
    Pose2d robotPose = new Pose2d(0,0,90);

    public Turret(OpMode opMode, String name) {
        super(opMode);
        this.name = name;

        this.localizer = new PinpointLocalizer(hardwareMap,robotPose);
        turret1 = hardwareMap.get(DcMotorEx.class,"turretDown");
        turret2 = hardwareMap.get(DcMotorEx.class,"turretUp");
        rot = hardwareMap.get(DcMotorEx.class,"turretRotate");


        s1 = hardwareMap.get(ServoImplEx.class,"leftHood");
        s2 = hardwareMap.get(ServoImplEx.class,"rightHood");
        s2.setDirection(Servo.Direction.REVERSE);


        turret1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        turret2.setDirection(DcMotorSimple.Direction.REVERSE);


        rotate = new RawEncoder(hardwareMap.get(DcMotorEx.class,"turretRotate"));
        voltageSensor = hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();



       /* turret = new Actuator(this, "turret", shootPidController.clone(), shootMotionProfile, 50, turret1,turret2) {




            @Override
            public boolean setTarget(double target) {
                this.target.set(target);
                return true;
            }


            public double getSpeed() {
                return (t1.getCurrentPosition().get(1)+t2.getCurrentPosition().get(1))/2.0;
            }
            @Override
            public DualNum<Time> getPosition() {
                return new DualNum<>(getSpeed(),0);
            }

            @Override
            public Command reset() {
                return Command.builder()
                        .init(() -> {
                            turret1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            turret2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                            turret1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            turret2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



                        })
                        .finished(() -> true)
                        .requires(Turret.this)
                        .build();
            }


        };*/

        hood = new ServoActuator(this, "hood", new TrapezoidalMotionProfile(10,16,16),s1,s2) {
            @Override
            public Command reset() {
                return Command.builder()
                        .init(() -> {
                            s1.setPosition(0);
                            s2.setPosition(0);
                        })
                        .finished(() -> true)
                        .requires(Turret.this)
                        .build();
            }
            public void setAngle(double angle) {
                angle = angle/100;
                //Angle between 0 - 60 degrees
                if(angle<HOOD_MIN_ANGLE)angle = HOOD_MIN_ANGLE;
                if(angle>HOOD_MAX_ANGLE)angle = HOOD_MAX_ANGLE;
                s1.setPosition(angle);
                s2.setPosition(angle);

            }

            @Override
            public boolean setTarget(double target) {
                target = target/100;
                if(target<HOOD_MIN_ANGLE)target = HOOD_MIN_ANGLE;
                if(target>HOOD_MAX_ANGLE)target = HOOD_MAX_ANGLE;

                this.target.set(target);
                return true;
            }
        };



    }
    public void setAngle(double angle){
        targetAngle = angle;
        setRotation();
    }
    public void setRotation(){
        double angle = targetAngle;
        time.reset();

        if (rotate.getCurrentPosition().get(0) <= tolerance || rotate.getCurrentPosition().get(0) >= -tolerance) {
            rot.setPower(0);}

        DualNum<Time> currentPosition = rotate.getCurrentPosition();

        final double distance = angle - currentPosition.get(0);
        DualNum<Time> mop = rotationalMotionProfile.getMotionState(Math.abs(distance),
                time.seconds());
        double power = rotationalPidController.update(mop.get(0) *Math.signum(distance),
                rotate.getCurrentPosition().get(0));

        rot.setTargetPosition((int) targetAngle);
        rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rot.setPower(power);

    }
    public double getRotation(){
        return rotate.getCurrentPosition().get(0)/8192/MOTOR_TO_TURRET_RATIO;
    }
    public Supplier<Boolean> atSpeed(){
        if(Math.abs(getSpeed()-targetSpeed)<=100)return ()->true;
        else return ()->false;
    }



    public void setTargetSpeed(double speed) {
        targetSpeed =  shootController.calculate(0,speed,shootMotionProfile.maxAcceleration)/6000;
        setShooter();
    }
    public void setShooter(){
        double power = targetSpeed;
        if (power < -1.0) power = -1.0;
        else if (power > 1.0) power = 1.0;
        turret1.setPower(power*(12/voltageSensor.getVoltage()));
        turret2.setPower(power*(12/voltageSensor.getVoltage()));
        telemetry.addData("shoot speed", power*6000);
    }
    public double getSpeed(){

        return turret1.getVelocity();
    }
    public double ppToAngle(PinpointLocalizer pinpoint,String TeamColor){
        Pose2d pose = pinpoint.getPose();
        double angle = 0;
        Pose2d corner = null;
        if(Objects.equals(TeamColor, "RED"))
            corner = new Pose2d(60,63,0);
        else if (Objects.equals(TeamColor, "BLUE"))
            corner = new Pose2d(-60,63,0);

        assert corner != null;
        double distance2origin = Math.abs(Math.sqrt(Math.pow(pose.position.x,2)+Math.pow(pose.position.y, 2)));
        double corner2origin =  Math.abs(Math.sqrt(Math.pow(corner.position.x,2)+Math.pow(corner.position.y,2)));
        angle = Math.atan2(distance2origin,corner2origin);

        return angle;

    }
    public Command trackCorner(PinpointLocalizer pinpoint,String TeamColor){
        return new InstantCommand(()->{setAngle(ppToAngle(pinpoint,TeamColor));});
    }
    public Command update(){
        return Command.builder()
                .update(()->{
                        setTargetSpeed(targetSpeed);
                        telemetry.addData("Error ", getSpeed() - targetSpeed);
                        setAngle(targetAngle);
                        telemetry.addData("Rotation error ", getRotation()-targetAngle);
                })
                .finished(()->{
                    if (Math.abs(getRotation() - targetAngle) <= tolerance&&getSpeed()-targetSpeed<=100.0)
                    {
                        setTargetSpeed(0);
                        return true;
                    }

                    else return false;
                })
                .build();

    }
    public Command reset(){
        return Command.builder()
                .update(()->{
                    turret1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    turret2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    rot.setTargetPosition(0);
                    rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rot.setPower(0.85);

                })
                .finished(()->{
                    return getRotation() == 0 && getSpeed() == 0;
                })
                .requires(this)
                .build();
    }


    }

