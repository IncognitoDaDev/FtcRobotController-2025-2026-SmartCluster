package org.firstinspires.ftc.teamcode.subsystem;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import java.util.function.Supplier;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.hardware.subsystem.ServoActuator;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.hardware.wrappers.Encoder;
import com.smartcluster.oracleftc.hardware.wrappers.RawEncoder;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

@Config
public class Spindex extends Subsystem {

    public static class CachedSensor
    {
        private ColorType.IdentityObject Front = ColorType.IdentityObject.EMPTY;;
        private ColorType.IdentityObject Left = ColorType.IdentityObject.EMPTY;;
        private ColorType.IdentityObject Right = ColorType.IdentityObject.EMPTY;;

        public void setFront(ColorType.IdentityObject obj)
        {
            if (obj != ColorType.IdentityObject.EMPTY)
                Front = obj;
        }

        public ColorType.IdentityObject getFront() { return Front; }

        public void setRight(ColorType.IdentityObject obj)
        {
            if (obj != ColorType.IdentityObject.EMPTY)
                Right = obj;
        }

        public ColorType.IdentityObject getRight() { return Right; }

        public void setLeft(ColorType.IdentityObject obj)
        {
            if (obj != ColorType.IdentityObject.EMPTY)
                Left = obj;
        }

        public ColorType.IdentityObject getLeft() { return Left; }

        public void reset()
        {
            Front = ColorType.IdentityObject.EMPTY;
            Left = ColorType.IdentityObject.EMPTY;
            Right = ColorType.IdentityObject.EMPTY;
        }
    }
    public final CRServoImplEx servoDexRight, servoDexLeft;
    public final ServoImplEx servoFlapperRight, servoFlapperLeft;
    public final RevColorSensorV3 rotaryColorSensorF;

    public final LynxI2cColorRangeSensor rotaryColorSensorR, rotaryColorSensorL;
    public final Encoder rotaryEncoder;

    public final OracleLynxVoltageSensor voltageSensor;

    public static PIDController rotaryPID = new PIDController(0.0065, 0.001, 0.00025);
    public static double Tolerance = 2;
    public static double ThirdTurn = 120; // 2750 degrees
    private double RightColorSensorOffset = 2.2;

    public double rotaryTargetPos = 0;

    public final ServoActuator flapper;
    public static double flapperDownVal = 0.51, flapperUpVal = 1.0;

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
                servoFlapperRight, servoFlapperRight)
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
    public Supplier<Boolean> flapperIsDown(){
        return () -> servoFlapperRight.getPosition()==flapperDownVal;
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
    public boolean IdentifyColor(RevColorSensorV3 sensor, ColorType[] ColorCheck)
    {
        for(ColorType check : ColorCheck)
        {
            int ok = 1;
            if (check.v3.RED_THRESHOLD[0] > sensor.red() || check.v3.RED_THRESHOLD[1] < sensor.red()) ok = 0;
            if (check.v3.GREEN_THRESHOLD[0] > sensor.green() || check.v3.GREEN_THRESHOLD[1] < sensor.green()) ok = 0;
            if (check.v3.BLUE_THRESHOLD[0] > sensor.blue() || check.v3.BLUE_THRESHOLD[1] < sensor.blue()) ok = 0;

            if (ok == 1) return true;
        }

        return false;
    }

    public boolean IdentifyColor(LynxI2cColorRangeSensor sensor, ColorType[] ColorCheck)
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
            if (check.v2.RED_THRESHOLD[0] > r || check.v2.RED_THRESHOLD[1] < r) ok = 0;
            if (check.v2.GREEN_THRESHOLD[0] > g || check.v2.GREEN_THRESHOLD[1] < g) ok = 0;
            if (check.v2.BLUE_THRESHOLD[0] > b || check.v2.BLUE_THRESHOLD[1] < b) ok = 0;

            if (ok == 1) return true;
        }

        return false;
    }

    public ColorType.IdentityObject IdentifyColor(RevColorSensorV3 sensor)
    {
        ColorType[] colorCheck = {ColorType.Purple, ColorType.Green, ColorType.Wall, ColorType.Nothing};
        for(ColorType check : colorCheck)
        {
            int ok = 1;
            if (check.v3.RED_THRESHOLD[0] > sensor.red() || check.v3.RED_THRESHOLD[1] < sensor.red()) ok = 0;
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

        ColorType[] colorCheck = {ColorType.Purple, ColorType.Green, ColorType.Wall, ColorType.Nothing};
        for(ColorType check : colorCheck)
        {
            int ok = 1;
            if (check.v2.RED_THRESHOLD[0] > r || check.v2.RED_THRESHOLD[1] < r) ok = 0;
            if (check.v2.GREEN_THRESHOLD[0] > g || check.v2.GREEN_THRESHOLD[1] < g) ok = 0;
            if (check.v2.BLUE_THRESHOLD[0] > b || check.v2.BLUE_THRESHOLD[1] < b) ok = 0;

            if (ok == 1) return check.identity;
        }

        return ColorType.IdentityObject.EMPTY;
    }

    public Command sortPurple()
    {
        if (cachedSensor.getLeft() == ColorType.IdentityObject.PURPLE)
        {
            return new InstantCommand(()->{this.SwitchMode(-1);});
        }
        if (cachedSensor.getRight() == ColorType.IdentityObject.PURPLE)
        {
            return new InstantCommand(()->{this.SwitchMode(1);});
        }
        if (cachedSensor.getFront() == ColorType.IdentityObject.PURPLE)
        {
            return new SequentialCommand(
                    new InstantCommand(()->{this.SwitchMode(-1);}),
                    NextSpace()
            );
        }
        return null;
    }

    public Command sortGreen()
    {
        if (cachedSensor.getLeft() == ColorType.IdentityObject.GREEN)
        {
            return new InstantCommand(()->{this.SwitchMode(-1);});
        }
        if (cachedSensor.getRight() == ColorType.IdentityObject.GREEN)
        {
            return new InstantCommand(()->{this.SwitchMode(1);});
        }
        if (cachedSensor.getFront() == ColorType.IdentityObject.GREEN)
        {
            return new SequentialCommand(
                    new InstantCommand(()->{this.SwitchMode(-1);}),
                    NextSpace()
            );
        }
        return null;
    }

    public Command sortAny()
    {
        if (cachedSensor.getLeft() == ColorType.IdentityObject.PURPLE || cachedSensor.getLeft() == ColorType.IdentityObject.GREEN)
        {
            return new InstantCommand(()->{this.SwitchMode(-1);});
        }
        if (cachedSensor.getRight() == ColorType.IdentityObject.PURPLE || cachedSensor.getRight() == ColorType.IdentityObject.GREEN)
        {
            return new InstantCommand(()->{this.SwitchMode(1);});
        }
        if (cachedSensor.getFront() == ColorType.IdentityObject.PURPLE || cachedSensor.getFront() == ColorType.IdentityObject.GREEN)
        {
            return new SequentialCommand(
                    new InstantCommand(()->{this.SwitchMode(-1);}),
                    NextSpace()
            );
        }
        return null;
    }

    public Command sortEmpty()
    {
        if (cachedSensor.getLeft() == ColorType.IdentityObject.EMPTY)
        {
            return new InstantCommand(()->{this.SwitchMode(-1);});
        }
        if (cachedSensor.getRight() == ColorType.IdentityObject.EMPTY)
        {
            return new InstantCommand(()->{this.SwitchMode(1);});
        }
        if (cachedSensor.getFront() == ColorType.IdentityObject.EMPTY)
        {
            return new SequentialCommand(
                    new InstantCommand(()->{this.SwitchMode(-1);}),
                    NextSpace()
            );
        }
        return null;
    }

    public void FixOrientationForIntake()
    {
        setTarget(rotaryTargetPos - rotaryTargetPos%ThirdTurn);
    }

    public Command NextSpace()
    {
        if (!flapperIsDown().get())
        {
            return new SequentialCommand(
                    new InstantCommand(this::FlapperDown),
                    new WaitCommand(200),
                    new InstantCommand(() -> {setTarget(rotaryTargetPos + ThirdTurn);})
            );
        }

        return new InstantCommand(() -> {setTarget(rotaryTargetPos + ThirdTurn);});
    }

    public Command PreviousSpace()
    {
        if (!flapperIsDown().get())
        {
            return new SequentialCommand(
                    new InstantCommand(this::FlapperDown),
                    new WaitCommand(150),
                    new InstantCommand(() -> {setTarget(rotaryTargetPos - ThirdTurn);})
            );
        }

        return new InstantCommand(() -> {setTarget(rotaryTargetPos - ThirdTurn);});
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
                .init(() ->
                {
                    setTarget(0);
                })
                .update(() ->
                {
                    setRotaryPower(rotaryPID.update(rotaryTargetPos, getPosition()));

                    telemetry.addData("CurrentPosition", getPosition());
                    telemetry.addData("TargetPosition", rotaryTargetPos);
                    telemetry.addData("ErrorDistance", getErrorDist());
                })
                .requires(this)
                .build();
    }

    public Command reset()
    {
        return new SequentialCommand(
                new InstantCommand(()-> {setRotaryPower(0);}),
                new InstantCommand(rotaryEncoder::reset));
    }

}
