package org.firstinspires.ftc.teamcode.subsystem;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.hardware.subsystem.ServoActuator;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.hardware.wrappers.Encoder;
import com.smartcluster.oracleftc.hardware.wrappers.RawEncoder;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

@Config
public class Spindex extends Subsystem {

    public static class CachedSensor
    {
        private ColorType.IdentityObject Front = ColorType.IdentityObject.EMPTY;
        private ColorType.IdentityObject Left = ColorType.IdentityObject.EMPTY;
        private ColorType.IdentityObject Right = ColorType.IdentityObject.EMPTY;

        public void setFront(ColorType.IdentityObject obj) { Front = obj; }

        public ColorType.IdentityObject getFront() { return Front; }

        public void setRight(ColorType.IdentityObject obj) { Right = obj; }

        public ColorType.IdentityObject getRight() { return Right; }

        public void setLeft(ColorType.IdentityObject obj) { Left = obj; }

        public ColorType.IdentityObject getLeft() { return Left; }

        public void reset()
        {
            Front = ColorType.IdentityObject.EMPTY;
            Left = ColorType.IdentityObject.EMPTY;
            Right = ColorType.IdentityObject.EMPTY;
        }

        public boolean setting_WALL = false;
//        public boolean setting_EMPTY = false;

        ColorType[] allColorChecks()
        {
            List<ColorType> col = new ArrayList<>();
            col.add(ColorType.Purple);
            col.add(ColorType.Green);
            
//            if (setting_EMPTY) col.add(ColorType.Nothing);
            if (setting_WALL) col.add(ColorType.Wall);

            return col.toArray(new ColorType[0]);
        }

    }
    public final CRServoImplEx servoDexRight, servoDexLeft;
    public final ServoImplEx servoFlapperRight;
    public final ServoImplEx servoFlapperLeft;
    public final RevColorSensorV3 rotaryColorSensorF;

    public final LynxI2cColorRangeSensor rotaryColorSensorR, rotaryColorSensorL;
    public final Encoder rotaryEncoder;

    public final OracleLynxVoltageSensor voltageSensor;

    public static PIDController rotaryPID = new PIDController(0.009, 0.000001, 0.00031); // new PIDController(0.007, 0.00005, 0.00025);

    public static TrapezoidalMotionProfile rotmp = new TrapezoidalMotionProfile(100,1000,1000);
    public static double Tolerance = 1;
    public static double ThirdTurn = 120; // 2750 degrees
    private final double RightColorSensorOffset = 1.4;
    public double currentPosition,target;
    public final ElapsedTime timer = new ElapsedTime();

    public double rotaryTargetPos = 0;

    public final ServoActuator flapper;
    public static double flapperDownVal = 1.0, flapperUpVal = 0.425;

    public CachedSensor cachedSensor;

    public Spindex (OpMode mode)
    {
        super(mode);
        servoDexRight = hardwareMap.get(CRServoImplEx.class, "dexRight");
        servoDexLeft = hardwareMap.get(CRServoImplEx.class, "dexLeft");
        servoFlapperRight=hardwareMap.get(ServoImplEx.class,"flapperRight");
        servoFlapperLeft=hardwareMap.get(ServoImplEx.class,"flapperLeft");
        rotaryColorSensorF = hardwareMap.get(RevColorSensorV3.class, "rotaryColorSensorF");
        rotaryColorSensorR = hardwareMap.get(LynxI2cColorRangeSensor.class, "rotaryColorSensorR");
        rotaryColorSensorL = hardwareMap.get(LynxI2cColorRangeSensor.class, "rotaryColorSensorL");
        rotaryEncoder = new RawEncoder(hardwareMap.get(DcMotorEx.class,"intakeMotor"));

        servoFlapperLeft.setDirection(Servo.Direction.REVERSE);

        cachedSensor = new CachedSensor();

        voltageSensor = hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();
        voltageSensor.setPolicy(OracleLynxVoltageSensor.OracleLynxVoltageSensorPolicy.CACHED);
        voltageSensor.setVoltageCacheFreshness(300);

        flapper = new ServoActuator(this, "flapper",
                new TrapezoidalMotionProfile(16, 20, 16),
                servoFlapperRight,servoFlapperLeft)
        {
            @Override
            public Command reset()
            {
                setTarget(flapperDownVal);
                return new InstantCommand(() ->
                {
                    servoFlapperRight.setPosition(this.target.get());
                    servoFlapperLeft.setPosition(this.target.get());
                });
            }

            @Override
            public boolean setTarget(double target)
            {
                this.target.set(target);
                return true;
            }
        };
    }

    public void FlapperDown()
    {
        flapper.setTarget(flapperDownVal);
    }
    public Boolean flapperIsDown(){
        return servoFlapperRight.getPosition()==flapperDownVal;
    }
    public void FlapperUp()
    {
        flapper.setTarget(flapperUpVal);
    }

    /*
     E mare nebunie aicia, dar este necesar pentru a rezolva paritatea datorita faptului
     ca avem doua tipuri senzori de culoare (V3 si V2 de la REV, sunt pozitionati diferiti)
     Sugestia mea este sa nu te uiti prea adanc. - R^2-M
    */
    public ColorType.IdentityObject IdentifyColor(RevColorSensorV3 sensor, ColorType[] ColorCheck)
    {
        for(ColorType check : ColorCheck)
        {
            int ok = 1;
            //if (check.v3.RED_THRESHOLD[0] > sensor.red() || check.v3.RED_THRESHOLD[1] < sensor.red()) ok = 0;
            if (check.v3.GREEN_THRESHOLD[0] > sensor.green() || check.v3.GREEN_THRESHOLD[1] < sensor.green()) ok = 0;
            if (check.v3.BLUE_THRESHOLD[0] > sensor.blue() || check.v3.BLUE_THRESHOLD[1] < sensor.blue()) ok = 0;

            if (ok == 1) return check.identity;
        }

        return ColorType.IdentityObject.EMPTY;
    }

    public ColorType.IdentityObject IdentifyColor(LynxI2cColorRangeSensor sensor, ColorType[] ColorCheck)
    {
        int r = sensor.red(), g = sensor.green(), b = sensor.blue();
        if (sensor == rotaryColorSensorR)
        {
            r = (int)(r*RightColorSensorOffset);
            g = (int)(g*RightColorSensorOffset);
            b = (int)(b*RightColorSensorOffset);
        }

        for(ColorType check : ColorCheck)
        {
            int ok = 1;
            //if (check.v2.RED_THRESHOLD[0] > r || check.v2.RED_THRESHOLD[1] < r) ok = 0;
            if (check.v2.GREEN_THRESHOLD[0] > g || check.v2.GREEN_THRESHOLD[1] < g) ok = 0;
            if (check.v2.BLUE_THRESHOLD[0] > b || check.v2.BLUE_THRESHOLD[1] < b) ok = 0;

            if (ok == 1) return check.identity;
        }

        return ColorType.IdentityObject.EMPTY;
    }

    public ColorType.IdentityObject IdentifyColor(RevColorSensorV3 sensor)
    {
        for(ColorType check : cachedSensor.allColorChecks())
        {
            int ok = 1;
            // if (check.v3.RED_THRESHOLD[0] > sensor.red() || check.v3.RED_THRESHOLD[1] < sensor.red()) ok = 0;
            if (check.v3.GREEN_THRESHOLD[0] > sensor.green() || check.v3.GREEN_THRESHOLD[1] < sensor.green()) ok = 0;
            if (check.v3.BLUE_THRESHOLD[0] > sensor.blue() || check.v3.BLUE_THRESHOLD[1] < sensor.blue()) ok = 0;

            if (ok == 1) return check.identity;
        }

        return ColorType.IdentityObject.EMPTY;
    }

    public ColorType.IdentityObject IdentifyColor(LynxI2cColorRangeSensor sensor)
    {
        int r = sensor.red(), g = sensor.green(), b = sensor.blue();
        if (sensor == rotaryColorSensorR)
        {
            r = (int)(r*RightColorSensorOffset);
            g = (int)(g*RightColorSensorOffset);
            b = (int)(b*RightColorSensorOffset);
        }

        for(ColorType check : cachedSensor.allColorChecks())
        {
            int ok = 1;
            // if (check.v2.RED_THRESHOLD[0] > r || check.v2.RED_THRESHOLD[1] < r) ok = 0;
            if (check.v2.GREEN_THRESHOLD[0] > g || check.v2.GREEN_THRESHOLD[1] < g) ok = 0;
            if (check.v2.BLUE_THRESHOLD[0] > b || check.v2.BLUE_THRESHOLD[1] < b) ok = 0;

            if (ok == 1) return check.identity;
        }

        return ColorType.IdentityObject.EMPTY;
    }

    public boolean sortPurple()
    {
        if (cachedSensor.getLeft() == ColorType.IdentityObject.PURPLE)
        {
            SwitchMode(-1);
            return true;
        }
        if (cachedSensor.getRight() == ColorType.IdentityObject.PURPLE)
        {
            SwitchMode(1);
            return true;
        }
        if (cachedSensor.getFront() == ColorType.IdentityObject.PURPLE)
        {
            SwitchMode(3);
            return true;
        }
        return false;
    }

    public boolean sortGreen()
    {
        if (cachedSensor.getLeft() == ColorType.IdentityObject.GREEN)
        {
            SwitchMode(-1);
            return true;
        }
        if (cachedSensor.getRight() == ColorType.IdentityObject.GREEN)
        {
            SwitchMode(1);
            return true;
        }
        if (cachedSensor.getFront() == ColorType.IdentityObject.GREEN)
        {
            SwitchMode(3);
            return true;
        }
        return false;
    }

    public boolean sortAny()
    {
        if (cachedSensor.getLeft() != ColorType.IdentityObject.EMPTY)
        {
            SwitchMode(-1);
            return true;
        }
        if (cachedSensor.getRight() != ColorType.IdentityObject.EMPTY)
        {
            SwitchMode(1);
            return true;
        }
        if (cachedSensor.getFront() != ColorType.IdentityObject.EMPTY)
        {
            SwitchMode(3);
            return true;
        }
        return false;
    }

    public boolean sortIntakeEmpty()
    {
        if (cachedSensor.getLeft() == ColorType.IdentityObject.EMPTY)
        {
            SwitchMode(-2);
            return true;
        }
        if (cachedSensor.getRight() == ColorType.IdentityObject.EMPTY)
        {
            SwitchMode(2);
            return true;
        }

        return false;
    }

    public Command NextSpace()
    {
        if (flapperIsDown())
        {
            return new SequentialCommand(
                            new InstantCommand(() -> {setTarget(rotaryTargetPos + ThirdTurn);}),
                            new WaitCommand(150)
                    );
            }
        else return new SequentialCommand(
                    new InstantCommand(this::FlapperDown),
                    new WaitCommand(200),
                    new InstantCommand(() -> {setTarget(rotaryTargetPos + ThirdTurn);}),
                    new WaitCommand(150)
        );
    }

    public Command PreviousSpace()
    {
        if (flapperIsDown()) {
            return new SequentialCommand(
                    new InstantCommand(() -> {
                        setTarget(rotaryTargetPos - ThirdTurn);
                    }),
                    new WaitCommand(150)
            );
        }
        else return new SequentialCommand(
                new InstantCommand(this::FlapperDown),
                new WaitCommand(200),
                new InstantCommand(() -> {setTarget(rotaryTargetPos - ThirdTurn);}),
                new WaitCommand(150)
        );
    }

    public void SwitchMode(int direction){
        setTarget(rotaryTargetPos - 60 * direction);
    }

    public void setTarget(double target)
    {
        rotaryTargetPos = target;
    }

    public double getErrorDist()
    {
        return Math.abs(rotaryTargetPos-getPosition());
    }

    public void setRotaryPower(double value)
    {
        if (value < -1.0) value = -1.0;
        else if (value > 1.0) value = 1.0;

        servoDexRight.setPower(value);
        servoDexLeft.setPower(value);
    }

    public double getPosition()
    {
        return rotaryEncoder.getCurrentPosition().get(0)/8192*360;
    }

    public final Command update()
    {
        return Command.builder()
                .init(()->{
                    currentPosition = getPosition();
                    target = rotaryTargetPos;
                    timer.reset();

                })
                .update(() ->
                {
                    if(currentPosition!=getPosition())currentPosition = getPosition();
                    if(target!=rotaryTargetPos)target = rotaryTargetPos;

                    final double distance = target - currentPosition;
                    DualNum<Time> mp = rotmp.getMotionState(Math.abs(distance),
                            timer.seconds());
                    double power = rotaryPID.update(mp.get(0) *Math.signum(distance)+currentPosition,
                            getPosition());
                    setRotaryPower(power);


                    telemetry.addData("CurrentSpindexerPosition", getPosition());
                    telemetry.addData("SpindexerTargetPosition", rotaryTargetPos);
                    telemetry.addData("Error Rotation", Math.abs(rotaryTargetPos-getPosition()));
                })
                .requires(this)
                .build();
    }

    public Command reset()
    {
        return new SequentialCommand(
                new InstantCommand(this::FlapperDown),
                new InstantCommand(()-> {setRotaryPower(0);}),
                new InstantCommand(rotaryEncoder::reset));
    }


    private double InitPos;
    private boolean StartOfCheck = false, EndOfCheck = false, EndOfReset = false;
    private ColorType.IdentityObject previousIdentity;
    public final Command resetRotary()
    {
        return new SequentialCommand(
                Command.builder()
                        .init(() ->
                        {
                            StartOfCheck = false;
                            EndOfCheck = false;
                            cachedSensor.setFront(IdentifyColor(rotaryColorSensorF, new ColorType[] {ColorType.Wall, ColorType.Nothing}));
                            previousIdentity = cachedSensor.getFront();

                            setRotaryPower(0.07); //Spin slowly to find the end of the object's identity
                        })
                        .update(() -> {
                            cachedSensor.setFront(IdentifyColor(rotaryColorSensorF, new ColorType[] {ColorType.Wall, ColorType.Nothing}));

                            if(previousIdentity != cachedSensor.getFront())
                            {
                                setRotaryPower(0.0); // HOLD UP
                                double currentPos = getPosition();
                                /*
                                    Ok the sensor detected a different identity object, we can now try to
                                    fix the rotary's position, however we have two cases to cover here
                                    1. Sensor initially faced the rotary's wall and 2. Sensor initially faced nothing.
                                 */

                                if (previousIdentity == ColorType.IdentityObject.WALL) {
                                    if (!StartOfCheck) {
                                        InitPos = currentPos;
                                        previousIdentity = cachedSensor.getFront(); // I'm Empty inside now!
                                        setRotaryPower(-0.07); // Reverse gear!!!
                                        StartOfCheck = true;
                                    } else {
                                        rotaryTargetPos = (currentPos + InitPos)/2; // Find the middle of the Wall
                                        EndOfCheck = true;
                                    }
                                }
                                else // Empty space... We don't want to reverse gear for this scenario ⚙️⚙️
                                {
                                    if (!StartOfCheck) {
                                        InitPos = getPosition();
                                        previousIdentity = cachedSensor.getFront(); // I'm a Wall now!
                                        setRotaryPower(0.07);
                                        StartOfCheck = true;
                                    } else {
                                        rotaryTargetPos = (currentPos + InitPos)/2; // Find the middle of the Wall
                                        EndOfCheck = true;
                                    }
                                }
                            }

                        })
                        .finished(() -> EndOfCheck)
                        .build(),

                // Now lets get into position and reset the Encoder!
                Command.builder()
                        .update(() ->
                        {
                            currentPosition = getPosition();
                            setRotaryPower(0.065*Math.signum(rotaryTargetPos - getPosition()));

                            if (getErrorDist() < Tolerance) // Reset once position is set
                            {
                                setRotaryPower(0.0);
                                rotaryTargetPos = 0;
                                rotaryEncoder.reset();
                            }

                            telemetry.addData("Error Rotation", rotaryTargetPos-getPosition());
                            telemetry.update();
                        })
                        .finished(() -> getErrorDist() < Tolerance)
                        .build()
        );
    }

}
