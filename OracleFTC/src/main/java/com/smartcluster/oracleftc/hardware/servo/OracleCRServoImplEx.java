package com.smartcluster.oracleftc.hardware.servo;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.Range;
import com.smartcluster.oracleftc.hardware.OmegaPowerCollector;

/**
 * CRServoEx provides access to extended functionality on continuous rotation
 * servos. Implementations support both the {@link CRServo} and {@link PwmControl} interfaces.
 */
public class OracleCRServoImplEx extends CRServoImplEx implements PwmControl {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    protected ServoControllerEx controllerEx;
    boolean isOnControlHub = true;
    OmegaPowerCollector powerCollector;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public OracleCRServoImplEx(ServoControllerEx controller, int portNumber, @NonNull ServoConfigurationType servoType)
    {
        this(controller, portNumber, DcMotor.Direction.FORWARD, servoType);
    }

    public OracleCRServoImplEx(ServoControllerEx controller, int portNumber, DcMotor.Direction direction, @NonNull ServoConfigurationType servoType)
    {
        super(controller, portNumber, servoType);
        this.direction = direction;
        this.controllerEx = controller;
        controllerEx.setServoType(portNumber, servoType);
    }

    @Override
    public void setPower(double power)
    {
        // For CR Servos on MR/HiTechnic hardware, internal positions relate to speed as follows:
        //
        //      0   == full speed reverse
        //      128 == stopped
        //      255 == full speed forward
        //
        if (this.direction == Direction.REVERSE) power = -power;
        power = Range.clip(power, apiPowerMin, apiPowerMax);
        power = Range.scale(power, apiPowerMin, apiPowerMax, apiServoPositionMin, apiServoPositionMax);
        powerCollector.bulkValues.setServoValue(getPortNumber() + (isOnControlHub ? 0 : 4), power);
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
