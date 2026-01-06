package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.ParallelCommand;
import com.smartcluster.oracleftc.commands.RaceCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.hardware.subsystem.CRActuator;
import com.smartcluster.oracleftc.hardware.subsystem.ServoActuator;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.hardware.wrappers.Encoder;
import com.smartcluster.oracleftc.hardware.wrappers.RawEncoder;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

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

    public static double flapperDownVal = 0.92, flapperUpVal = 0.44;

    public static double dexTarget = 0;


    public static PIDController spindexerPID = new PIDController(0.009, 0.000001, 0.00031);
    public static TrapezoidalMotionProfile spindexerMotionProfile = new TrapezoidalMotionProfile(100,1000,1000);
    public enum ArtifactColor{
        GREEN,
        PURPLE,
        EMPTY
    }
    public static class StorageState {

        // Order is F R L, clockwise
        public ArtifactColor[] Slot = {ArtifactColor.EMPTY, ArtifactColor.EMPTY, ArtifactColor.EMPTY};
        public ArtifactColor[] Order = {ArtifactColor.EMPTY, ArtifactColor.EMPTY, ArtifactColor.EMPTY};

        // 0 for none; 1 for Clockwise, -1 for Anticlockwise
        public int OuttakeFacing = 0;

        public void appendBallIntake(ArtifactColor obj)
        {
            Slot[0] = obj;
        }

        public void removeBallOuttake()
        {
            if (OuttakeFacing == -1) Slot[1] = ArtifactColor.EMPTY;
            if (OuttakeFacing == 1) Slot[2] = ArtifactColor.EMPTY;
        }

        private void next() // Clockwise
        {
            ArtifactColor Slot0 = Slot[0];
            Slot[0] = Slot[1];
            Slot[1] = Slot[2];
            Slot[2] = Slot0;
        }

        public void previous() // Anticlockwise
        {
            ArtifactColor Slot0 = Slot[0];
            Slot[0] = Slot[2];
            Slot[2] = Slot[1];
            Slot[1] = Slot0;
        }

        public Supplier<Boolean> isFull(){
            for(int i = 0;i<3;i++)
                if (Slot[i] == ArtifactColor.EMPTY)
                    return () -> false;

            return () -> true;
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
                return new InstantCommand(() ->
                {
                    setTarget(flapperDownVal);
                    flapperRight.setPosition(this.target.get());
                    flapperLeft.setPosition(this.target.get());
                });
            }

            @Override
            public boolean setTarget(double target)
            {
//                if(target >= flapperDownVal || target <= flapperUpVal) return false;
                this.target.set(target);
                return true;
            }
        };


        spindexer = new CRActuator(this, "spindexer",  spindexerPID, spindexerMotionProfile, 6.0, spindexLeft,spindexRight) {
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

                spindexEncoder.reset();
                setTarget(0);
                return move(new AtomicReference<>(0.0));
            }
        };
    }
    public Command flapperUp() { return flapper.move(new AtomicReference<>(flapperUpVal)); }
    public Command flapperDown() { return flapper.move(new AtomicReference<>(flapperDownVal)); }

    public Storage.ArtifactColor identifyObj(RevColorSensorV3 sensor)
    {
        NormalizedRGBA data = sensor.getNormalizedColors();

        if (data.alpha*256 > 230) { //Something exists... and it's a ball
            if (data.green * 256 > 8.3) // Checking if is GREEN
                return Storage.ArtifactColor.GREEN;
            else
                return (Storage.ArtifactColor.PURPLE); // Must be PURPLE then
        }

        return Storage.ArtifactColor.EMPTY;
    }

    public Storage.ArtifactColor identifyObjFrontSensor()
    {
        return identifyObj(frontColorSensor);
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

    public Command BallToOuttake()
    {
        return new SequentialCommand(
                flapperUp(),
                new InstantCommand(() -> storage.removeBallOuttake()),
                new WaitCommand(100),
                flapperDown()
        );
    }

    // Positive is clockwise (Negative is anticlockwise)
    // Use -+1 for 60 degrees for outtake
    public Command outtakeMode(int Direction)
    {
        return Command.builder()
                .init(()->{
                    storage.OuttakeFacing += Direction;
                    spindexer.setTarget(spindexer.getTarget()+ Direction*60);
                })
                .finished(()->Math.abs(spindexer.getPosition().get(0)-spindexer.getTarget())<spindexer.tolerance)
                .build();
    }

    public Command intakeMode()
    {
        // Use only after using outtake mode :pray:
        return Command.builder()
                .init(()->{
                    int Direction = 0;

                    if (storage.OuttakeFacing == -1) Direction = 1;
                    else if (storage.OuttakeFacing == 1) Direction = -1;
                    storage.OuttakeFacing += Direction;

                    spindexer.setTarget(spindexer.getTarget()+Direction*60);
                })
                .finished(()->Math.abs(spindexer.getPosition().get(0)-spindexer.getTarget())<spindexer.tolerance)
                .build();
    }

    private double originalPos = 0;

    public Command Shake(double duration, double distanceFromOriginalPos)
    {
        final ElapsedTime timer = new ElapsedTime();
        return Command.builder()
                .init(() ->
                {
                    timer.reset();
                    originalPos = spindexer.getPosition().get(0);
                })
                .update(() ->
                {
                    if (Math.abs(spindexer.getPosition().get(0)-spindexer.getTarget())<1.0)
                    {
                        if (spindexer.getTarget() > originalPos)
                            spindexer.setTarget(originalPos + distanceFromOriginalPos);
                        else
                            spindexer.setTarget(originalPos - distanceFromOriginalPos);
                    }
                })
                .finished(() -> timer.milliseconds() > duration)
                .build();
    }


    private int greenCount = 0, purpleCount = 0;
    public Command SlotCheck(double minDurationScan)
    {
        final ElapsedTime timer = new ElapsedTime();
        return Command.builder()
                .init(() ->
                {
                    timer.reset();
                    greenCount = 0;
                    purpleCount = 0;
                })
                .update(() ->
                {
                    ArtifactColor dataScanned = identifyObjFrontSensor();
                    if (dataScanned != ArtifactColor.EMPTY)
                    {
                        if (dataScanned == ArtifactColor.PURPLE) purpleCount++;
                        else greenCount ++;

                        storage.Slot[0] = dataScanned;
                    }
                })
                .finished(() ->
                {
                    if (storage.Slot[0] != ArtifactColor.EMPTY && timer.milliseconds() > minDurationScan)
                    {
                        if (greenCount > purpleCount) storage.Slot[0] = ArtifactColor.GREEN;
                        else storage.Slot[0] = ArtifactColor.PURPLE;
                        return true;
                    }

                    else if (timer.milliseconds() > 600)
                        return true; // You're taking too long...

                    return false; //Keep going
                })
                .build();
    }

    public Command SingleSlotCheck()
    {
        return new SequentialCommand(
                new ParallelCommand(
                        Shake(300, 8),
                        SlotCheck(100)
                ),
                new InstantCommand(() -> spindexer.setTarget(originalPos))
        );
    }
    public Command routineBallInspection()
    {
        return new SequentialCommand(

                intakeMode(),
                SingleSlotCheck(),
                nextBall(),
                SingleSlotCheck(),
                nextBall(),
                SingleSlotCheck()
        );
    }

    public Command sort(ArtifactColor ball) // Assuming you're in outtake mode
    {
        return Command.builder()
                .init(()->{
                    if (storage.OuttakeFacing == -1)
                    {
                        if (storage.Slot[1] == ball); // Ball is here
                        else if (storage.Slot[2] == ball)
                        {
                            storage.next();
                            spindexer.setTarget(spindexer.getTarget()+120);
                        }
                        else if (storage.Slot[0] == ball)
                        {
                            storage.previous();
                            spindexer.setTarget(spindexer.getTarget()-120);
                        }
                    }
                    else if (storage.OuttakeFacing == 1)
                    {
                        if (storage.Slot[2] == ball); // Ball is here
                        else if (storage.Slot[0] == ball)
                        {
                            storage.next();
                            spindexer.setTarget(spindexer.getTarget()+120);
                        }
                        else if (storage.Slot[1] == ball)
                        {
                            storage.previous();
                            spindexer.setTarget(spindexer.getTarget()-120);
                        }
                    }
                })
                .update(() -> telemetry.addData("Looking for", ball))
                .finished(()->Math.abs(spindexer.getPosition().get(0)-spindexer.getTarget())<spindexer.tolerance)
                .build();
    }

    public Command sort(int order) // Assuming you're in outtake mode
    {
        return Command.builder()
                .init(()->{
                    if (storage.OuttakeFacing == -1)
                    {
                        if (storage.Slot[1] == storage.Order[order]); // Ball is here
                        else if (storage.Slot[2] == storage.Order[order])
                        {
                            storage.next();
                            spindexer.setTarget(spindexer.getTarget()+120);
                        }
                        else if (storage.Slot[0] == storage.Order[order])
                        {
                            storage.previous();
                            spindexer.setTarget(spindexer.getTarget()-120);
                        }
                    }
                    else if (storage.OuttakeFacing == 1)
                    {
                        if (storage.Slot[2] == storage.Order[order]); // Ball is here
                        else if (storage.Slot[0] == storage.Order[order])
                        {
                            storage.next();
                            spindexer.setTarget(spindexer.getTarget()+120);
                        }
                        else if (storage.Slot[1] == storage.Order[order])
                        {
                            storage.previous();
                            spindexer.setTarget(spindexer.getTarget()-120);
                        }
                    }
                })
                .update(() -> telemetry.addData("Looking for", storage.Order[order]))
                .finished(()->Math.abs(spindexer.getPosition().get(0)-spindexer.getTarget())<spindexer.tolerance)
                .build();
    }

    public Command update()
    {
        return new ParallelCommand(
                flapper.update(),
                spindexer.update()
        );
    }

}
