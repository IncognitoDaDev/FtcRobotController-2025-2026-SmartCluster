package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.hardware.subsystem.ServoActuator;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

@Config
public class Spindex extends Subsystem {
    public final CRServoImplEx servoDexRight, servoDexLeft;
    public final ServoImplEx servoFlapperRight;
    public final RevColorSensorV3 rotaryColorSensorF;

    public final LynxI2cColorRangeSensor rotaryColorSensorR, rotaryColorSensorL;
    public final DcMotorEx rotaryEncoder;

    public final OracleLynxVoltageSensor voltageSensor;

    //private OracleLynxVoltageSensor voltageSensor;
    public static PIDController rotaryPID = new PIDController(0.00012, 0.00000025, 0);//new PIDController(0.00032, 0.000000015, 0.000013, 0);
    public static double Tolerance = 40;
    public static double ThirdTurn = 2750; // 120 degrees

    public double previousValueAnalog = 0;
    public double rotaryCurrentPos = 0;
    public double rotaryTargetPos = 0;

    public final ServoActuator flapper;
    public static double flapperDownVal = 0.51, flapperUpVal = 1.0;

    public Spindex (OpMode mode)
    {
        super(mode);

        servoDexRight = hardwareMap.get(CRServoImplEx.class, "dexRight");
        servoDexLeft = hardwareMap.get(CRServoImplEx.class, "dexLeft");
        servoFlapperRight=hardwareMap.get(ServoImplEx.class,"flapperRight");
        //servoFlapperLeft=hardwareMap.get(ServoImplEx.class,"flapperLeft");
        rotaryColorSensorF = hardwareMap.get(RevColorSensorV3.class, "rotaryColorSensorF");
        rotaryColorSensorR = hardwareMap.get(LynxI2cColorRangeSensor.class, "rotaryColorSensorR");
        rotaryColorSensorL = hardwareMap.get(LynxI2cColorRangeSensor.class, "rotaryColorSensorL");
        rotaryEncoder = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        rotaryEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        voltageSensor = hardwareMap.getAll(OracleLynxVoltageSensor.class).iterator().next();
        voltageSensor.setPolicy(OracleLynxVoltageSensor.OracleLynxVoltageSensorPolicy.CACHED);
        voltageSensor.setVoltageCacheFreshness(300);

        flapper = new ServoActuator(this, "flapper", new TrapezoidalMotionProfile(12, 16, 12), servoFlapperRight)
        {
            @Override
            public Command reset()
            {
                setTarget(flapperDownVal);
                return new InstantCommand(() ->
                {
                    servoFlapperRight.setPosition(this.target.get());
                    //servoFlapperLeft.setPosition(this.target.get());
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
            r *= 2;
            g *= 2;
            b *= 2;
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
            r *= 2;
            g *= 2;
            b *= 2;
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

    public boolean sortPurple()
    {
        if (IdentifyColor(rotaryColorSensorL, new ColorType[]{ColorType.Purple}))
        {
            setTarget(rotaryTargetPos - ThirdTurn/2 - rotaryTargetPos%ThirdTurn);
            return true;
        }
        if (IdentifyColor(rotaryColorSensorR, new ColorType[]{ColorType.Purple}))
        {
            setTarget(rotaryTargetPos + ThirdTurn/2 - rotaryTargetPos%ThirdTurn);
            return true;
        }
        if (IdentifyColor(rotaryColorSensorF, new ColorType[]{ColorType.Purple}))
        {
            setTarget(rotaryTargetPos + ThirdTurn*1.5 - rotaryTargetPos%ThirdTurn);
            return true;
        }
        return false;
    }

    public boolean sortGreen()
    {
        if (IdentifyColor(rotaryColorSensorL, new ColorType[]{ColorType.Green}))
        {
            setTarget(rotaryTargetPos - ThirdTurn/2 - rotaryTargetPos%ThirdTurn);
            return true;
        }
        if (IdentifyColor(rotaryColorSensorR, new ColorType[]{ColorType.Green}))
        {
            setTarget(rotaryTargetPos + ThirdTurn/2 - rotaryTargetPos%ThirdTurn);
            return true;
        }
        if (IdentifyColor(rotaryColorSensorF, new ColorType[]{ColorType.Green}))
        {
            setTarget(rotaryTargetPos + ThirdTurn*1.5 - rotaryTargetPos%ThirdTurn);
            return true;
        }
        return false;
    }

    public boolean sortAny()
    {
        if (IdentifyColor(rotaryColorSensorL, new ColorType[]{ColorType.Green, ColorType.Purple}))
        {
            setTarget(rotaryTargetPos - ThirdTurn/2 - rotaryTargetPos%ThirdTurn);
            return true;
        }
        if (IdentifyColor(rotaryColorSensorR, new ColorType[]{ColorType.Green, ColorType.Purple}))
        {
            setTarget(rotaryTargetPos + ThirdTurn/2 - rotaryTargetPos%ThirdTurn);
            return true;
        }
        if (IdentifyColor(rotaryColorSensorF, new ColorType[]{ColorType.Green, ColorType.Purple}))
        {
            setTarget(rotaryTargetPos + ThirdTurn*1.5 - rotaryTargetPos%ThirdTurn);
            return true;
        }
        return false;
    }

    public boolean sortEmpty()
    {
        if (IdentifyColor(rotaryColorSensorL, new ColorType[]{ColorType.Nothing}))
        {
            setTarget(rotaryTargetPos - ThirdTurn - rotaryTargetPos%ThirdTurn);
            return true;
        }
        if (IdentifyColor(rotaryColorSensorR, new ColorType[]{ColorType.Nothing}))
        {
            setTarget(rotaryTargetPos + ThirdTurn - rotaryTargetPos%ThirdTurn);
            return true;
        }
//        if (IdentifyColor(rotaryColorSensorF, new ColorType[]{ColorType.Nothing}))
//        {
//            setTarget(rotaryCurrentPos + ThirdTurn*2);
//            return true;
//        } // We want to move ball to an empty space from front position
        return false;
    }


    public void FixOrientationForIntake()
    {
        setTarget(rotaryTargetPos - rotaryTargetPos%ThirdTurn);
    }

    public void setTarget(double target)
    {
        rotaryTargetPos = target;
    }

    public void setRotaryPower(double value)
    {
        double voltage = voltageSensor.getVoltage();

        if (value < -1.0) value = -1.0;
        else if (value > 1.0) value = 1.0;

        servoDexRight.setPower(value*(12.0/voltage));
        servoDexLeft.setPower(value*(12.0/voltage));
    }

    public double getPosition()
    {
        return rotaryEncoder.getCurrentPosition();
    }

    public void updateRotaryPosition()
    {
        setRotaryPower(rotaryPID.update(rotaryTargetPos, rotaryEncoder.getCurrentPosition()));
        telemetry.addData("RotaryPosError", getPosition() - rotaryTargetPos);
    }

    public Command update()
    {
        return Command.builder()
                .update(() ->
                {
                    setRotaryPower(rotaryPID.update(rotaryTargetPos, rotaryEncoder.getCurrentPosition()));
                    telemetry.addData("Error", getPosition() - rotaryTargetPos);
                })
                .finished(() ->
                        {
                           if (Math.abs(getPosition() - rotaryTargetPos) <= Tolerance)
                           {
                               setRotaryPower(0);
                               return true;
                           }
                           else return false;
                        })
                .requires(this)
                .build();
    }

    public Command reset()
    {
        return new InstantCommand(()-> {setRotaryPower(0);});
    }

}
