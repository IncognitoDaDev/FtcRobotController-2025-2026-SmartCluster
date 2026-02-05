package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
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
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.hardware.OmegaPowerCollector;
import com.smartcluster.oracleftc.hardware.subsystem.OmegaCRActuator;
import com.smartcluster.oracleftc.hardware.subsystem.OmegaServoActuator;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.hardware.wrappers.Encoder;
import com.smartcluster.oracleftc.hardware.wrappers.OmegaCRServoImplEx;
import com.smartcluster.oracleftc.hardware.wrappers.OmegaServoImplEx;
import com.smartcluster.oracleftc.hardware.wrappers.RawEncoder;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class Storage extends Subsystem {

    private final OmegaCRServoImplEx spindexRight, spindexLeft;
    private final OmegaServoImplEx flapperRight, flapperLeft;
    private final RevColorSensorV3 frontColorSensor;
    public final Encoder spindexEncoder;
    public static TrapezoidalMotionProfile flapperMotionProfile = new TrapezoidalMotionProfile(16, 16, 16);
    public final OmegaServoActuator flapper;
    public final OmegaCRActuator spindexer;

    public static double flapperDownVal = 0.23, flapperUpVal = 0.5;

    public static PIDController spindexerPID = new PIDController(0.0045, 0.00014, 0.00014);
    public static TrapezoidalMotionProfile spindexerMotionProfile = new TrapezoidalMotionProfile(3000,2500,850);
    public enum ArtifactColor{
        GREEN,
        PURPLE,
        EMPTY
    }
    public static class StorageState {

        // Order is F R L, clockwise
        public ArtifactColor[] Slot = {ArtifactColor.EMPTY, ArtifactColor.EMPTY, ArtifactColor.EMPTY};
        public static ArtifactColor[] Order = {ArtifactColor.EMPTY, ArtifactColor.EMPTY, ArtifactColor.EMPTY};

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

        public boolean isFull()
        {
            for(int i = 0; i<3; i++)
                if (Slot[i] == ArtifactColor.EMPTY)
                    return false;

            return true;
        }

        public boolean isEmpty()
        {
            for(int i = 0; i<3; i++)
                if (Slot[i] == ArtifactColor.EMPTY)
                    return false;

            return true;
        }
    }

    public StorageState storage = new StorageState();

    public Storage(OpMode opMode, OmegaPowerCollector powerCollector) {
        super(opMode);
        spindexRight = new OmegaCRServoImplEx(hardwareMap.get(CRServoImplEx.class, "dexRight"), powerCollector, true);
        spindexLeft = new OmegaCRServoImplEx(hardwareMap.get(CRServoImplEx.class, "dexLeft"), powerCollector, true);
        flapperRight = new OmegaServoImplEx(hardwareMap.get(ServoImplEx.class,"flapperRight"), powerCollector, true);
        flapperLeft = new OmegaServoImplEx(hardwareMap.get(ServoImplEx.class,"flapperLeft"), powerCollector, true);

        spindexEncoder = new RawEncoder(hardwareMap.get(DcMotorEx.class,"frontRight"));

        frontColorSensor = hardwareMap.get(RevColorSensorV3.class, "rotaryColorSensorF");

        flapperLeft.getServo().setDirection(Servo.Direction.REVERSE);

//        frontColorSensor_Purple = hardwareMap.get(DigitalChannelImpl.class, "rotaryColorSensorF_Purple");
//        frontColorSensor_Green = hardwareMap.get(DigitalChannelImpl.class, "rotaryColorSensorF_Green");

        flapper = new OmegaServoActuator(this, "flapper", flapperMotionProfile,flapperLeft,flapperRight)
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
                this.target.set(target);
                return true;
            }
        };


        spindexer = new OmegaCRActuator(this, "spindexer",  spindexerPID, spindexerMotionProfile, 4.0, spindexLeft, spindexRight) {
            @Override
            public boolean setTarget(double target) {
                this.ManualSetFromPosition(getPosition().get(0));
                this.target.set(target);
                return true;
            }

            @Override
            public DualNum<Time> getPosition() {
                return spindexEncoder.getCurrentPosition().div(8192).times(360);
            }

            @Override
            public Command reset() {
                return new SequentialCommand(
                        new InstantCommand(spindexEncoder::reset),
                        spindexer.move(new AtomicReference<Double>(0.0))
                        );
            }
        };
    }
    public Command flapperUp() { return flapper.move(new AtomicReference<>(flapperUpVal)); }
    public Command flapperDown() { return flapper.move(new AtomicReference<>(flapperDownVal)); }
    public Storage.ArtifactColor identifyObj()
    {
//        if (frontColorSensor_Purple.getState()) return ArtifactColor.PURPLE;
//        if (frontColorSensor_Green.getState()) return ArtifactColor.GREEN;

        NormalizedRGBA data = frontColorSensor.getNormalizedColors();
        frontColorSensor.setGain(2);

        if (frontColorSensor.getDistance(DistanceUnit.MM)<152) // Is something in front?
        {
            if (data.blue*256 < data.green*256) return ArtifactColor.GREEN;
            if(data.blue*256> data.green*256)return ArtifactColor.PURPLE;
        }

        return Storage.ArtifactColor.EMPTY;
    }

    public Command nextBall() // Clockwise
    {
        return Command.builder()
                .init(()->{
                    storage.next();
                    spindexer.setTarget(spindexer.getTarget()+120);
                })
                .finished(spindexer.isNotInMotion())
                .build();

    }
    public Command previousBall() //Anticlockwise
    {
        return Command.builder()
                .init(()->{
                    storage.previous();
                    spindexer.setTarget(spindexer.getTarget()-120);
                })
                .finished(spindexer.isNotInMotion())
                .build();
    }

    public Command BallToOuttake()
    {
        return new SequentialCommand(
                flapperUp(),
                flapperDown(),
                new InstantCommand(() -> storage.removeBallOuttake())
        );
    }

    // Positive is clockwise (Negative is anticlockwise)
    // Use -+1 for 60 degrees for outtake
    public Command outtakeMode(int Direction)
    {
        return Command.builder()
                .init(()->{
                    storage.OuttakeFacing += Direction;
                    spindexer.setTarget(spindexer.getTarget() + Direction*60);
                })
                .finished(spindexer.isNotInMotion())
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

                    spindexer.setTarget(spindexer.getTarget() + Direction*60);
                })
                .finished(spindexer.isNotInMotion())
                .build();
    }

    public Command BroPleaseStopItsEmpty()
    {
        return Command.builder()
                .finished(() -> storage.isEmpty())
                .build();
    }

    public Command SlotCheck(double maxDuration)
    {
        final ElapsedTime timer = new ElapsedTime();
        return Command.builder()
                .init(timer::reset)
                .update(() ->
                {
                    ArtifactColor dataScanned = identifyObj();
                    if (dataScanned != ArtifactColor.EMPTY)
                        storage.Slot[0] = dataScanned;
                })
                .finished(() -> storage.Slot[0] != ArtifactColor.EMPTY || timer.milliseconds() > maxDuration)
                .build();
    }
    public boolean hasBall(){
        return frontColorSensor.getDistance(DistanceUnit.MM) < 152;
    }

    public Command WaitForBall(int maxBall, double maxDuration)
    {
        final ElapsedTime timer = new ElapsedTime();

        AtomicBoolean isSpin = new AtomicBoolean(false);
        AtomicInteger ballCount = new AtomicInteger(0);
        return Command.builder()
                .init(timer::reset)
                .update(() -> {
                    if (isSpin.get())
                    {
                        if (spindexer.isNotInMotion().get())
                            isSpin.set(false);
                    } else { // Spindexer doesn't need to move, so scan all you can!
                        ArtifactColor dataScanned = identifyObj();
                        if (dataScanned != ArtifactColor.EMPTY) {
                            ballCount.getAndIncrement();
                            storage.Slot[0] = dataScanned;

                            if (!storage.isFull())
                            {
                                isSpin.set(true);
                                spindexer.setTarget(spindexer.getPosition().get(0)+120);
                                storage.next();
                            }
                        }
                    }
                })
                .finished(() -> storage.isFull()
                        || timer.milliseconds() > maxDuration
                        || ballCount.get() >= maxBall)
                .build();
    }
    public Command distanceSwitch(int maxBall, double maxDuration)
    {
        final ElapsedTime timer = new ElapsedTime();

        AtomicBoolean isSpin = new AtomicBoolean(false);
        AtomicInteger ballCount = new AtomicInteger(0);
        return Command.builder()
                .init(timer::reset)
                .update(() -> {
                    if (isSpin.get())
                    {
                        if (spindexer.isNotInMotion().get())
                            isSpin.set(false);
                    } else { // Spindexer doesn't need to move, so scan all you can!
                        ArtifactColor dataScanned = identifyObj();
                        if(frontColorSensor.getDistance(DistanceUnit.MM)<152) {
                            ballCount.getAndIncrement();
//                            storage.Slot[0] = dataScanned;

                            if (!storage.isFull())
                            {
                                isSpin.set(true);
                                spindexer.setTarget(spindexer.getPosition().get(0)+120);
                                storage.next();
                            }
                        }
                    }
                })
                .finished(() -> storage.isFull()
                        || timer.milliseconds() > maxDuration
                        || ballCount.get() >= maxBall)
                .build();
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
                .finished(spindexer.isNotInMotion())
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
                .finished(spindexer.isNotInMotion())
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