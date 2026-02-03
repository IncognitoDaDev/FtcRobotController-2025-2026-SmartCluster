package com.smartcluster.oracleftc.hardware.servo;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.Range;
import com.smartcluster.oracleftc.hardware.OmegaPowerCollector;

/**
 * ServoImplEx provides access to extended functionality on servos. Instances support
 * both the {@link Servo} and the {@link PwmControl} interfaces.
 */
public class OracleServoImplEx extends ServoImplEx implements PwmControl
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    protected ServoControllerEx controllerEx;
    boolean isOnControlHub = true;
    OmegaPowerCollector powerCollector;


    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public OracleServoImplEx(ServoControllerEx controller, int portNumber, @NonNull ServoConfigurationType servoType)
    {
        this(controller, portNumber, Direction.FORWARD, servoType);
    }

    public OracleServoImplEx(ServoControllerEx controller, int portNumber, Direction direction, @NonNull ServoConfigurationType servoType)
    {
        super(controller, portNumber, servoType);
        this.direction = direction;
        this.controllerEx = controller;
        controllerEx.setServoType(portNumber, servoType);
    }

    @Override
    synchronized public void setPosition(double position) {
        position = Range.clip(position, MIN_POSITION, MAX_POSITION);
        if (direction == Direction.REVERSE) position = reverse(position);
        double scaled = Range.scale(position, MIN_POSITION, MAX_POSITION, limitPositionMin, limitPositionMax);
        powerCollector.bulkValues.setServoValue(getPortNumber() + (isOnControlHub ? 0 : 4), scaled);
    }

    private double reverse(double position) {
        return MAX_POSITION - position + MIN_POSITION;
    }

    public void setDestination(OmegaPowerCollector powerCollector, boolean isPluggedIntoControlHub)
    {
        this.powerCollector = powerCollector;
        isOnControlHub = isPluggedIntoControlHub;
    }

    //----------------------------------------------------------------------------------------------
    // PwmControl
    //----------------------------------------------------------------------------------------------

    @Override
    public void setPwmRange(PwmRange range)
    {
        controllerEx.setServoPwmRange(this.getPortNumber(), range);
    }

    @Override
    public PwmRange getPwmRange()
    {
        return controllerEx.getServoPwmRange(this.getPortNumber());
    }

    @Override
    public void setPwmEnable()
    {
        controllerEx.setServoPwmEnable(this.getPortNumber());
    }

    @Override
    public void setPwmDisable()
    {
        controllerEx.setServoPwmDisable(this.getPortNumber());
    }

    @Override
    public boolean isPwmEnabled()
    {
        return controllerEx.isServoPwmEnabled(this.getPortNumber());
    }
}
