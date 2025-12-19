package org.firstinspires.ftc.teamcode.subsystem;



import static com.ThermalEquilibrium.homeostasis.Utils.MathUtils.normalizeAngle;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;

import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

@Config
public class Turret extends Subsystem {

    final double HOOD_MIN_ANGLE = 0.0;   // Example: Angle when servo is at position 0.0
    final double HOOD_MAX_ANGLE = 60.0;
    private static final double MIN_TURRET_ANGLE = -180;
    private static final double MAX_TURRET_ANGLE = 180;
    public final double MOTOR_TO_TURRET_RATIO =  260.0/48;
    public final double ENCODER_TICKS_PER_ROTATION = 384.5*MOTOR_TO_TURRET_RATIO;
    public final double ENCODER_TICKS_PER_DEGREE = ENCODER_TICKS_PER_ROTATION/360;
    public static PIDFCoefficients coef = new PIDFCoefficients(260,0,0,12);
    private final String name;
    public DcMotorEx turret1,turret2,rot;
    public ServoImplEx s1,s2;
    public static TrapezoidalMotionProfile shootMotionProfile = new TrapezoidalMotionProfile(6000,40000,20000);
    public static PIDFController SHPIDF = new PIDFController(coef);
    public static PIDController rotationalPidController = new PIDController(0.002,0.0002,0.0003);
    public static TrapezoidalMotionProfile rotationalMotionProfile = new TrapezoidalMotionProfile(6000,1000,1000);
    public final Encoder rotate;
    public double[] Speed ={0,4500,5000,6000};
    private final OracleLynxVoltageSensor voltageSensor;
    public static double tolerance = 3;

    private final ElapsedTime time = new ElapsedTime();
    public static double Kv = 1.1;
    public static double Ka = 0;
    public static double Ks = 0.001;
    FeedforwardCoefficients coefficients = new FeedforwardCoefficients(Kv,Ka,Ks);
    BasicFeedforward shootController = new BasicFeedforward(coefficients);
    public double targetSpeed = 0;
    public double targetAngle = 0;
    public double currentAngle,currentSpeed;


    public GoBildaPinpointDriver pinpoint;
//    public Actuator turret;
//    public Actuator rotation;
    public ServoActuator hood;
    Pose2d robotPose = new Pose2d(0,0,90);

    public Turret(OpMode opMode, String name) {
        super(opMode);
        this.name = name;

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        turret1 = hardwareMap.get(DcMotorEx.class, "turretDown");
        turret2 = hardwareMap.get(DcMotorEx.class, "turretUp");
        rot = hardwareMap.get(DcMotorEx.class, "turretRotate");


        s1 = hardwareMap.get(ServoImplEx.class, "leftHood");
        s2 = hardwareMap.get(ServoImplEx.class, "rightHood");
        s2.setDirection(Servo.Direction.REVERSE);


        rot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        turret2.setDirection(DcMotorSimple.Direction.REVERSE);
        turret1.setDirection(DcMotorSimple.Direction.FORWARD);
        turret2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        com.qualcomm.robotcore.hardware.PIDFCoefficients pidfCoefficients = new com.qualcomm.robotcore.hardware.PIDFCoefficients(260,0,0,12);

        turret2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        rotate = new RawEncoder(hardwareMap.get(DcMotorEx.class, "turretRotate"));
        voltageSensor = hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();

        rot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hood = new ServoActuator(this, "hood", new TrapezoidalMotionProfile(10,16,16),s1,s2) {
            @Override
            public Command reset() {
                setTarget(0.9);
                return new InstantCommand(() ->
                {
                    s1.setPosition(this.target.get());
                    s2.setPosition(this.target.get());
                });
            }

            @Override
            public boolean setTarget(double target) {
                this.target.set(target);
                return true;
            }
        };
    }
    public double HoodAngle(Pose2d pose,String team){
        double angle = 0;
        Pose2d corner = null;
        if(Objects.equals(team, "BLUE"))
            corner = new Pose2d(60,63, -45);
        else if (Objects.equals(team, "RED"))
            corner = new Pose2d(-60,63,-45);
        assert corner != null;
        double distance = Math.sqrt(Math.pow(corner.position.y-pose.position.x,2)+Math.pow(corner.position.x-pose.position.x,2));

        return angle;
    }

    public void setAngle(double angle){
        targetAngle = angle;
        setRotation();
    }
    public void setRotation(){
        double angle = targetAngle;

        if (Math.abs(getRotation()-targetAngle)<=tolerance) {
            rot.setPower(0);}
        else {
            DualNum<Time> currentPosition = rotate.getCurrentPosition();

            final double distance = angle - currentPosition.get(0);
            DualNum<Time> mop = rotationalMotionProfile.getMotionState(Math.abs(distance),
                    time.seconds());
            double power = rotationalPidController.update(mop.get(0) * Math.signum(distance),
                    rotate.getCurrentPosition().get(0));

            rot.setTargetPosition((int) targetAngle);
            rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rot.setPower(power);
        }
    }
    public double getRotation(){
        return rotate.getCurrentPosition().get(0)/ENCODER_TICKS_PER_DEGREE;
    }

    public double getCurrentSpeed(){
        return turret1.getVelocity()/8192;
    }
    public void setShooterSpeed(double speed) {
        targetSpeed = speed*8192/6000;
        if (targetSpeed < 0.0) targetSpeed = 0.0;
        if (targetSpeed > 6000.0) targetSpeed = 6000.0;
        turret2.setVelocity(targetSpeed);
        turret1.setPower(turret2.getPower());
    }
    public void ppToAngle(Pose2d pose, String TeamColor){
        Pose2d corner = null;
        if(Objects.equals(TeamColor, "BLUE"))
            corner = new Pose2d(60,63, -45);
        else if (Objects.equals(TeamColor, "RED"))
            corner = new Pose2d(-60,63,-45);

        assert corner != null;
        double dx = corner.position.x - pose.position.x;
        double dy = corner.position.y - pose.position.y;

        double angleToCorner = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = Math.toDegrees(pose.heading.log());

        double turretAngle;
        if (robotHeading <= 180)  turretAngle = -(90 - angleToCorner + robotHeading); // angleToCorner - robotHeading
        else turretAngle = angleToCorner - robotHeading;

        telemetry.addLine("dx: " + dx + " dy: " + dy);
        telemetry.addLine("angleToCorner: " + angleToCorner);
        telemetry.addLine("turretAngle: " + turretAngle);
        time.reset();

        setAngle(turretAngle);
    }


    public Command update(){
        return Command.builder()
                .init(()->{
                    currentAngle = getRotation();
                    time.reset();
                })
                .update(()->{
                    if(currentAngle!=getRotation())currentAngle = getRotation();
                    double distance = targetAngle - currentAngle;
                    DualNum<Time> mp = rotationalMotionProfile.getMotionState(Math.abs(distance),
                            time.seconds());


                    double power = rotationalPidController.update(mp.get(0) *Math.signum(distance)+currentAngle,
                            rotate.getCurrentPosition().get(0));
                    rot.setPower(power);

                    telemetry.addData("PP to Corner angle",targetAngle);
                    telemetry.addData("Rotation error ", targetAngle);
                    telemetry.addData("Rotational position", getRotation());
                    telemetry.addData("Current speed",getCurrentSpeed());
                })
                .build();
    }
    public Command ppUpdate(Localizer pp){
        return Command.builder()
                .init(()->{
                    currentAngle = getRotation();
                    time.reset();
                })
                .update(()->{
                    if(currentAngle!=getRotation())currentAngle = getRotation();
                    pp.update();
                    ppToAngle(pp.getPose(),"RED");
                    double distance = targetAngle - currentAngle;
                    DualNum<Time> mp = rotationalMotionProfile.getMotionState(Math.abs(distance),
                            time.seconds());
                    double power = rotationalPidController.update(mp.get(0) *Math.signum(distance)+currentAngle,
                            rotate.getCurrentPosition().get(0));
                    rot.setPower(power);
                    telemetry.addData("PP to Corner angle",targetAngle);
                    telemetry.addData("Rotation error ", getRotation()-targetAngle);
                })
                .build();
    }

    public Command reset(){
        return Command.builder()
                .update(()->{
                    turret1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    turret2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rotate.reset(360);

                    rot.setTargetPosition(0);
                    rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rot.setPower(0.85);

                })
                .finished(()-> Math.abs(getRotation()) <= tolerance)
                .requires(this)
                .build();
    }


    }

