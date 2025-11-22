package org.firstinspires.ftc.teamcode.subsystem;



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

import java.util.concurrent.atomic.AtomicReference;


public class Turret extends Subsystem {

    final double HOOD_MIN_ANGLE = 0.0;   // Example: Angle when servo is at position 0.0
    final double HOOD_MAX_ANGLE = 60.0;
    public final double MAX_FLYWHEEL_TICKS_PER_SEC = 2800;
    public final double MOTOR_TO_TURRET_RATIO =  260.0/48;
    public final double ENCODER_TICKS_PER_ROTATION = 384.5*MOTOR_TO_TURRET_RATIO;
    public final double ENCODER_TICKS_PER_DEGREE = ENCODER_TICKS_PER_ROTATION/360;
    private final String name;
    public DcMotorEx turret1,turret2,rot;
    public ServoImplEx s1,s2;
    public static PIDController shootPidController = new PIDController(0.0,0.0,0.0);
    public static TrapezoidalMotionProfile shootMotionProfile = new TrapezoidalMotionProfile(6000,40000,20000);
    public static PIDController rotationalPidController = new PIDController(0.0,0.0,0.0);
    public static TrapezoidalMotionProfile rotationalMotionProfile = new TrapezoidalMotionProfile(6000,40000,20000);
    private final Encoder t1,t2,rotate;
    public double[] Speed ={0,4500,5000,6000};
    private OracleLynxVoltageSensor voltageSensor;

    private final ElapsedTime time = new ElapsedTime();


    public Actuator turret;
    public Actuator rotation;
    public ServoActuator hood;

    public Turret(OpMode opMode, String name) {
        super(opMode);
        this.name = name;
        turret1 = hardwareMap.get(DcMotorEx.class,"turretDown");
        turret2 = hardwareMap.get(DcMotorEx.class,"turretUp");
        rot = hardwareMap.get(DcMotorEx.class,"turretRotate");


        s1 = hardwareMap.get(ServoImplEx.class,"leftHood");
        s2 = hardwareMap.get(ServoImplEx.class,"rightHood");
        s2.setDirection(Servo.Direction.REVERSE);


        turret1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        turret1.setDirection(DcMotorSimple.Direction.REVERSE);

        t1 = new RawEncoder(hardwareMap.get(DcMotorEx.class,"turretDown"));
        t1.setDirection(Encoder.Direction.REVERSE);
        t2 = new RawEncoder(hardwareMap.get(DcMotorEx.class,"turretUp"));
        rotate = new RawEncoder(hardwareMap.get(DcMotorEx.class,"turretRotate"));


        turret = new Actuator(this, "turret", shootPidController.clone(), shootMotionProfile, 50, turret1,turret2) {


            public Command setSpeed(double speed) {
                return new InstantCommand(() -> {
                    setTarget(speed);
                });


            }

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
                return new DualNum<Time>(getSpeed(),0);
            }

            @Override
            public Command reset() {
                return Command.builder()
                        .init(() -> {
                            turret1.setPower(0);
                            turret2.setPower(0);


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
        rotation = new Actuator(this,"rotation",rotationalPidController.clone(),rotationalMotionProfile,10,rot) {
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


                // Convert the raw ticks into degrees by dividing by our constant.
                // This automatically scales both the position and the velocity.
                return rotate.getCurrentPosition().div(ENCODER_TICKS_PER_DEGREE);
            }

            @Override
            public Command reset() {
                // This command should move the turret to a known "zero" position.
                // This could be against a hard stop or aligned with a limit switch.
                this.target.set(0.0);
                AtomicReference<DualNum<Time>> lastPosition = new AtomicReference<>(getPosition());
                ElapsedTime resetTime = new ElapsedTime();

                return Command.builder()
                        .init(() -> {
                            // Move towards the hard stop. Use negative power if going the other way.
                            // Use a gentle power to avoid damage.
                            rot.setPower(0.3);
                            resetTime.reset();
                        })
                        .finished(() -> {
                            DualNum<Time> currentPosition = getPosition();
                            // Check if the turret has stopped moving (stalled against the stop)
                            if (Math.abs(currentPosition.minus(lastPosition.get()).size()) < 0.5) { // 0.5 degree tolerance
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
            public double setAngle(double angle) {
                //Angle between 0 - 60 degrees
                setTarget(angle);
                return angle;
            }

            @Override
            public boolean setTarget(double target) {
                if(target<HOOD_MIN_ANGLE)target = HOOD_MIN_ANGLE;
                if(target>HOOD_MAX_ANGLE)target = HOOD_MAX_ANGLE;

                this.target.set(target);
                return true;
            }
        };




    }


}
