package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.hardware.subsystem.Actuator;
import com.smartcluster.oracleftc.hardware.subsystem.CRActuator;
import com.smartcluster.oracleftc.hardware.subsystem.ServoActuator;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.hardware.subsystem.SubsystemFlavor;
import com.smartcluster.oracleftc.hardware.wrappers.Encoder;
import com.smartcluster.oracleftc.hardware.wrappers.RawEncoder;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class Storage extends Subsystem {

    private final CRServoImplEx spindexRight, spindexLeft;
    private final ServoImplEx flapperRight, flapperLeft;
    private final LynxI2cColorRangeSensor leftColorSensor, rightColorSensor;
    private final RevColorSensorV3 frontColorSensor;
    public final Encoder spindexEncoder;
    public static TrapezoidalMotionProfile flapperMotionProfile = new TrapezoidalMotionProfile(16, 20, 16);
    public final ServoActuator flapper;
    public final CRActuator spindexer;

    public static double flapperDownVal = 1.0, flapperUpVal = 0.425;

    public static double dexTarget = 0;


    public static PIDController spindexerPID = new PIDController(0.009, 0.000001, 0.00031);
    public static TrapezoidalMotionProfile spindexerMotionProfile = new TrapezoidalMotionProfile(100,1000,1000);
    public enum ArtifactColor{
        GREEN,
        PURPLE,
        EMPTY
    }
    public class StorageState {

        // Order is F R L, clockwise
        public ArtifactColor[] Storage = {ArtifactColor.EMPTY, ArtifactColor.EMPTY, ArtifactColor.EMPTY};

        // 0 for none; 1 for Clockwise, -1 for Anticlockwise
        public int OuttakeFacing = 0;

        public void appendBallIntake(ArtifactColor obj)
        {
            Storage[0] = obj;
        }

        public void removeBallOuttake()
        {
            if (OuttakeFacing == -1) Storage[1] = ArtifactColor.EMPTY;
            if (OuttakeFacing == 1) Storage[2] = ArtifactColor.EMPTY;
        }

        public void next() // Clockwise
        {
            ArtifactColor Temp = Storage[0];
            Storage[0] = Storage[1];
            Storage[1] = Storage[2];
            Storage[2] = Temp;
        }

        public void previous() // Anticlockwise
        {
            ArtifactColor Temp = Storage[2];
            Storage[2] = Storage[1];
            Storage[1] = Storage[0];
            Storage[0] = Temp;
        }
    }

    public StorageState storage = new StorageState();

    public Storage(OpMode opMode) {
        super(opMode);
        spindexRight = hardwareMap.get(CRServoImplEx.class, "dexRight");
        spindexLeft = hardwareMap.get(CRServoImplEx.class, "dexLeft");
        flapperRight=hardwareMap.get(ServoImplEx.class,"flapperRight");
        flapperLeft=hardwareMap.get(ServoImplEx.class,"flapperLeft");
        spindexEncoder = new RawEncoder(hardwareMap.get(DcMotorEx.class,"intakeMotor"));
        flapperLeft.setDirection(Servo.Direction.REVERSE);
        frontColorSensor = hardwareMap.get(RevColorSensorV3.class, "rotaryColorSensorF");
        rightColorSensor = hardwareMap.get(LynxI2cColorRangeSensor.class, "rotaryColorSensorR");
        leftColorSensor = hardwareMap.get(LynxI2cColorRangeSensor.class, "rotaryColorSensorL");
        flapper = new ServoActuator(this, "flapper", flapperMotionProfile, flapperRight,flapperLeft)
        {
            @Override
            public Command reset()
            {
                setTarget(flapperDownVal);
                return new InstantCommand(() ->
                {
                    flapperRight.setPosition(this.target.get());
                    flapperLeft.setPosition(this.target.get());
                });
            }

            @Override
            public boolean setTarget(double target)
            {
                if(target>flapperDownVal || target<flapperUpVal) return false;
                this.target.set(target);
                return true;
            }
        };


        spindexer = new CRActuator(this, "spindexer",  spindexerPID, spindexerMotionProfile, 10.0, spindexLeft,spindexRight) {
            @Override
            public boolean setTarget(double target) {
                this.target.set(target);
                return true;
            }

            @Override
            public DualNum<Time> getPosition() {
                return spindexEncoder.getCurrentPosition().div(8192).times(360);
            }

            @Override
            public Command reset() {
                setTarget(0);
                return move(new AtomicReference<>(60.0));
            }
        };
    }
    public Command flapperUp()
    {
        return flapper.move(new AtomicReference<>(flapperUpVal));
    }
    public Command flapperDown()
    {
        return flapper.move(new AtomicReference<>(flapperDownVal));
    }

    public Command nextBall() // Clockwise
    {
        return Command.builder()
                .init(()->{
                    storage.next();
                    spindexer.setTarget(spindexer.getTarget()+120);
                })
                .finished(()->Math.abs(spindexer.getPosition().get(0)-spindexer.getTarget())<spindexer.tolerance)
                .build();

    }
    public Command previousBall() //Anticlockwise
    {
        return Command.builder()
                .init(()->{
                    storage.previous();
                    spindexer.setTarget(spindexer.getTarget()-120);
                })
                .finished(()->Math.abs(spindexer.getPosition().get(0)-spindexer.getTarget())<spindexer.tolerance)
                .build();
    }

    // Positive is clockwise? (Negative is anticlockwise?)
    // Use -+1 for 60 degrees for outtake (Call twice, once at start to set for outtake, and once at end for intake)
    public Command outtakeMode(int Direction)
    {
        storage.OuttakeFacing += Direction;
        return Command.builder()
                .init(()->{
                    spindexer.setTarget(spindexer.getTarget()+ Direction*60);
                })
                .finished(()->Math.abs(spindexer.getPosition().get(0)-spindexer.getTarget())<spindexer.tolerance)
                .build();
    }

    public Command sort(ArtifactColor ball) // Assuming you're in outtake mode
    {
        if (storage.OuttakeFacing == -1) // Ok the ball in slot 1 is right below the outtake
        {
            if (storage.Storage[1] == ball)
                return outtakeMode(0); // Does nothing...
            if (storage.Storage[2] == ball)
                return nextBall();
            if (storage.Storage[0] == ball)
                return previousBall();
        }
        else // Ok the ball in slot 2 is right below the outtake
        {
            if (storage.Storage[2] == ball)
                return outtakeMode(0); // Does nothing...
            if (storage.Storage[1] == ball)
                return nextBall();
            if (storage.Storage[0] == ball)
                return previousBall();
        }
        return outtakeMode(0); //Oh well, lets do nothing?
    }

    public Command update()
    {
        return new ParallelCommand(
                flapper.update(),
                spindexer.update()
        );
    }

}
