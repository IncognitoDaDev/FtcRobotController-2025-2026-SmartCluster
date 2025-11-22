package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.hardware.OracleLynxVoltageSensor;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.math.control.PIDController;

@Config
public class Spindex extends Subsystem {
    public final CRServoImplEx servoDexRight, servoDexLeft;
    public final AnalogInput rotaryAnalog; // Uses ServoDexRight
    public final RevColorSensorV3 rotaryColorSensor;

    //private OracleLynxVoltageSensor voltageSensor;
    public static PIDController rotaryPID = new PIDController(0.002, 0.002, 0.002, 0);
    public static double Tolerance = 5;


    public double previousValueAnalog = 0;
    public double rotaryCurrentPos = 0;
    public double rotaryTargetPos = 0;
    public Spindex (OpMode mode)
    {
        super(mode);

        servoDexRight = hardwareMap.get(CRServoImplEx.class, "servodexright");
        servoDexLeft = hardwareMap.get(CRServoImplEx.class, "servodexleft");
        //servoTongue = mode.hardwareMap.get(ServoImplEx.class, "tongue");

        rotaryAnalog = hardwareMap.get(AnalogInput.class, "rotaryAnalog");
        rotaryColorSensor = hardwareMap.get(RevColorSensorV3.class, "rotaryColorSensor");
    }
    public enum BallColor
    {
        ANY,
        PURPLE,
        GREEN
    }

    /*public Command SearchColor(BallColor desiredColor)
    {
        return Command.builder()
                .update(() ->
                {
                    switch(desiredColor)
                    {
                        case PURPLE:
                            if (rotaryColorSensor.)
                            break;
                    }
                })
    } */

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
        rotaryCurrentPos = rotaryAnalog.getVoltage() * 355 / 3.3;
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
        rotaryAnalog.getVoltage();
        return new InstantCommand(()-> {setRotaryPower(0);});
    }





}
