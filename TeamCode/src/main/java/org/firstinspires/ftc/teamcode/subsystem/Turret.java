package org.firstinspires.ftc.teamcode.subsystem;



import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.hardware.subsystem.Actuator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

import java.util.concurrent.atomic.AtomicReference;


public class Turret extends Subsystem {
    public final double MAX_FLYWHEEL_TICKS_PER_SEC = 2800;
    public final double MOTOR_TO_TURRET_RATIO =  260.0/48;
    public final double ENCODER_TICKS_PER_ROTATION = 384.5*MOTOR_TO_TURRET_RATIO;
    public final double ENCODER_TICKS_PER_DEGREE = ENCODER_TICKS_PER_ROTATION/360;

    public DcMotorEx turret1,turret2,rot;
    public Servo s1,s2;
    public static PIDController shootPidController = new PIDController(0.0,0.0,0.0);
    public static TrapezoidalMotionProfile shootMotionProfile = new TrapezoidalMotionProfile(6000,40000,20000);
    public static PIDController rotationalPidController = new PIDController(0.0,0.0,0.0);
    public static TrapezoidalMotionProfile rotationalMotionProfile = new TrapezoidalMotionProfile(6000,40000,20000);
    private Encoder t1,t2,rotate;
    public double[] Speed ={0,4500,5000,6000};

    public Actuator turret;
    public Actuator rotation;
    public enum Phases{
        NONE,CLOSE,MIDDLE,FAR
    }


    public Turret(OpMode opMode) {
        super(opMode);
        turret1 = hardwareMap.get(DcMotorEx.class,"turretDown");
        turret2 = hardwareMap.get(DcMotorEx.class,"turretUp");
        rot = hardwareMap.get(DcMotorEx.class,"turretRotate");

        s1 = hardwareMap.get(Servo.class,"hoodServo1");
        s2 = hardwareMap.get(Servo.class,"hoodServo2");

        turret1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        turret1.setDirection(DcMotorSimple.Direction.REVERSE);

        t1 = new RawEncoder(hardwareMap.get(DcMotorEx.class,"turretDown"));
        t1.setDirection(DcMotorSimple.Direction.REVERSE);
        t2 = new RawEncoder(hardwareMap.get(DcMotorEx.class,"turretUp"));
        rotate = new RawEncoder(hardwareMap.get(DcMotorEx.class,"turretRotate"));


        turret = new Actuator(this, "turret", shootPidController, shootMotionProfile, 50, turret1,turret2) {

            public void setPower(double power) {
                if (power > 1.0) power = 1.0;
                if (power < -1.0) power = -1.0;


                double targetVelocity = power * MAX_FLYWHEEL_TICKS_PER_SEC;


                this.setTarget(targetVelocity);
            }

            public double setSpeed(Phases phases,double speed) {
                switch(phases){
                case NONE:
                    speed = Speed[0];
                    break;
                case CLOSE:
                    speed = Speed[1];
                    break;
                case MIDDLE:
                    speed = Speed[2];
                    break;
                case FAR:
                    speed = Speed[3];
                    break;
                default:
                    speed = Speed[0];
                    break;

                }
                setTarget(speed);
                return speed;
            }

            @Override
            public boolean setTarget(double target) {
                this.target.set(target);
                return false;
            }
            public DualNum<Time> getSpeed() {
                double pose = (double) (t1.getPositionAndVelocity().velocity + t2.getPositionAndVelocity().velocity) /2;
                return new DualNum<Time>(pose,0);
            }
            @Override
            public DualNum<Time> getPosition() {
                double pose = (double) (t1.getPositionAndVelocity().position +  t2.getPositionAndVelocity().position) /2/8192*360;
                return new DualNum<Time>(90.0,0).minus(pose);
            }

            @Override
            public Command reset() {
                return Command.builder()
                        .init(() -> {
                            turret1.setPower(0);
                            turret2.setPower(0);

                            // 2. Set the motor mode to STOP_AND_RESET_ENCODER
                            // This is the official FTC SDK way to reset motor encoders.
                            turret1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            turret2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                            turret1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            turret2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                        })
                        .finished(() -> true)
                        .requires(Turret.this)
                        .build();
            }


        };
        rotation = new Actuator(this,"rotation",rotationalPidController,rotationalMotionProfile,10,rot) {
            @Override
            public boolean setTarget(double target) {
                // Clamp the target angle to prevent wire twist and damage
                if (target < -180) {
                    target = 180;
                }
                if (target > 180) {
                    target = -180;
                }
                this.target.set(target);
                return true; // Signal that the target was accepted
            }

            @Override
            public DualNum<Time> getPosition() {
                // Read the raw encoder ticks and convert them to degrees
                return new DualNum<Time>(rotate.getPositionAndVelocity().position / ENCODER_TICKS_PER_DEGREE,0);
            }

            @Override
            public Command reset() {
                // This command should move the turret to a known "zero" position.
                // This could be against a hard stop or aligned with a limit switch.
                this.target.set(0.0);
                AtomicReference<Double> lastPosition = new AtomicReference<>(getPosition().get(0));
                ElapsedTime resetTime = new ElapsedTime();

                return Command.builder()
                        .init(() -> {
                            // Move towards the hard stop. Use negative power if going the other way.
                            // Use a gentle power to avoid damage.
                            rot.setPower(0.3);
                            resetTime.reset();
                        })
                        .finished(() -> {
                            double currentPosition = getPosition().get(0);
                            // Check if the turret has stopped moving (stalled against the stop)
                            if (Math.abs(currentPosition - lastPosition.get()) < 0.5) { // 0.5 degree tolerance
                                // If stalled, wait for a timeout to confirm
                                return resetTime.milliseconds() > 500; // Wait 500ms
                            } else {
                                // If still moving, update the position and reset the timer
                                lastPosition.set(currentPosition);
                                resetTime.reset();
                            }
                            return false;
                        })
                        .end((interrupted) -> {
                            // THIS IS THE GOAL: Reset the encoder to zero at the hard stop.
                            rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            // Set a small holding power or stop the motor
                            rot.setPower(0);
                        })
                        .requires(Turret.this)
                        .build();
            }
        };




    }

}
