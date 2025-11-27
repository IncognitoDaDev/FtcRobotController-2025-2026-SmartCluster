package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.hardware.subsystem.ServoActuator;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.math.control.PIDController;
import com.smartcluster.oracleftc.math.control.TrapezoidalMotionProfile;

@Config
public class Spindex extends Subsystem {
    public final CRServoImplEx servoDexRight, servoDexLeft;
    public final ServoImplEx servoFlapperRight;
    public final RevColorSensorV3 rotaryColorSensor;

    //private OracleLynxVoltageSensor voltageSensor;
    public static PIDController rotaryPID = new PIDController(0.002, 0.002, 0.002, 0);
    public static double Tolerance = 5;


    public double previousValueAnalog = 0;
    public double rotaryCurrentPos = 0;
    public double rotaryTargetPos = 0;

    public final ServoActuator flapper;
    public static double flapperDownVal = 0.85, flapperUpVal = 0.4;

    public Spindex (OpMode mode)
    {
        super(mode);

        servoDexRight = hardwareMap.get(CRServoImplEx.class, "dexRight");
        servoDexLeft = hardwareMap.get(CRServoImplEx.class, "dexLeft");
        servoFlapperRight=hardwareMap.get(ServoImplEx.class,"flapperRight");
        //servoFlapperLeft=hardwareMap.get(ServoImplEx.class,"flapperLeft");
        rotaryColorSensor = hardwareMap.get(RevColorSensorV3.class, "rotaryColorSensor");

        //servoFlapperRight.setDirection(Servo.Direction.REVERSE);
        //servoFlapperLeft.setDirection(Servo.Direction.REVERSE);

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
    public enum BallColor
    {
        EMPTY,
        PURPLE,
        GREEN
    }

    public void FlapperDown()
    {
        flapper.setTarget(flapperDownVal);
    }

    public void FlapperUp()
    {
        flapper.setTarget(flapperUpVal);
    }


    public BallColor IdentifyColor()
    {
        if (rotaryColorSensor.green()>=Color.Green.GREEN_THRESHOLD[0]){
            return BallColor.GREEN;

        }
        if (rotaryColorSensor.blue()>=Color.Purple.BLUE_THRESHOLD[0]){
            return BallColor.PURPLE;
        }
        return BallColor.EMPTY;
    }

    public void setTarget(double target)
    {
        if (target < 0) rotaryTargetPos = 360 - target%360;
        rotaryTargetPos = target%360;
    }

    public void setRotaryPower(double value)
    {
        if (value < -1.0) value = -1.0;
        else if (value > 1.0) value = 1.0;

        servoDexRight.setPower(value);
        servoDexLeft.setPower(value);
    }

    public double getRotaryPosition()
    {
        //rotaryCurrentPos = rotaryAnalog.getVoltage() * 355 / 3.3;
        return rotaryCurrentPos;
    }

    public Command update()
    {
        return Command.builder()
                .update(() ->
                {
                    setRotaryPower(rotaryPID.update(rotaryTargetPos, getRotaryPosition()));
                })
                .finished(() ->
                        {
                           if (Math.abs(getRotaryPosition() - rotaryTargetPos) <= Tolerance)
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
