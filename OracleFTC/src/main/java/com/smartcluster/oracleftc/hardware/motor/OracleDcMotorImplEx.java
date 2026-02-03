package com.smartcluster.oracleftc.hardware.motor;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.smartcluster.oracleftc.hardware.OmegaPowerCollector;

public class OracleDcMotorImplEx extends DcMotorImplEx implements DcMotorEx
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    DcMotorControllerEx controllerEx;
    int                 targetPositionTolerance = LynxConstants.DEFAULT_TARGET_POSITION_TOLERANCE;
    boolean isOnControlHub = true;

    OmegaPowerCollector powerCollector;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public OracleDcMotorImplEx(DcMotorController controller, int portNumber)
    {
        this(controller, portNumber, Direction.FORWARD);
    }

    public OracleDcMotorImplEx(DcMotorController controller, int portNumber, Direction direction)
    {
        this(controller, portNumber, direction, MotorConfigurationType.getUnspecifiedMotorType());
    }

    public OracleDcMotorImplEx(DcMotorController controller, int portNumber, Direction direction, @NonNull MotorConfigurationType motorType)
    {
        super(controller, portNumber, direction, motorType);
        this.controllerEx = (DcMotorControllerEx)controller;
    }

    @Override
    public void setPower(double power)
    {
        if (getMode() == RunMode.RUN_TO_POSITION) {
            power = Math.abs(power);
        } else {
            power = adjustPower(power);
        }
        powerCollector.bulkValues.setDcMotorValue(getPortNumber() + (isOnControlHub ? 0 : 4), power);
    }

    public void setDestination(OmegaPowerCollector powerCollector, boolean isPluggedIntoControlHub)
    {
        this.powerCollector = powerCollector;
        isOnControlHub = isPluggedIntoControlHub;
    }

    //----------------------------------------------------------------------------------------------
    // DcMotorEx interface
    //----------------------------------------------------------------------------------------------

//    @Override
//    public void setMotorEnable()
//    {
//        controllerEx.setMotorEnable(this.getPortNumber());
//    }
//
//    @Override
//    public void setMotorDisable()
//    {
//        controllerEx.setMotorDisable(this.getPortNumber());
//    }
//
//    @Override
//    public boolean isMotorEnabled()
//    {
//        return controllerEx.isMotorEnabled(this.getPortNumber());
//    }
//
//    @Override public synchronized void setVelocity(double angularRate)
//    {
//        angularRate = adjustAngularRate(angularRate);
//        controllerEx.setMotorVelocity(getPortNumber(), angularRate);
//    }
//
////    @Override public synchronized void setVelocity(double angularRate, AngleUnit unit)
////    {
////        angularRate = adjustAngularRate(angularRate);
////        controllerEx.setMotorVelocity(getPortNumber(), angularRate, unit);
////    }
//
//    @Override public synchronized double getVelocity()
//    {
//        double angularRate = controllerEx.getMotorVelocity(this.getPortNumber());
//        angularRate = adjustAngularRate(angularRate);
//        return angularRate;
//    }
//
//    @Override
//    public synchronized double getVelocity(AngleUnit unit)
//    {
//        double angularRate = controllerEx.getMotorVelocity(this.getPortNumber(), unit);
//        angularRate = adjustAngularRate(angularRate);
//        return angularRate;
//    }
//
//    protected double adjustAngularRate(double angularRate)
//    {
//        if (getOperationalDirection() == Direction.REVERSE) angularRate = -angularRate;
//        return angularRate;
//    }
//
//    @Override public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients)
//    {
//        controllerEx.setPIDCoefficients(this.getPortNumber(), mode, pidCoefficients);
//    }
//
//    @Override public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients)
//    {
//        controllerEx.setPIDFCoefficients(this.getPortNumber(), mode, pidfCoefficients);
//    }
//
//    @Override public void setVelocityPIDFCoefficients(double p, double i, double d, double f)
//    {
//        setPIDFCoefficients(RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f, MotorControlAlgorithm.PIDF));
//    }
//
//    @Override public void setPositionPIDFCoefficients(double p)
//    {
//        setPIDFCoefficients(RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, 0, 0, 0, MotorControlAlgorithm.PIDF));
//    }
//
//    @Override public PIDCoefficients getPIDCoefficients(RunMode mode)
//    {
//        return controllerEx.getPIDCoefficients(this.getPortNumber(), mode);
//    }
//
//    @Override public PIDFCoefficients getPIDFCoefficients(RunMode mode)
//    {
//        return controllerEx.getPIDFCoefficients(this.getPortNumber(), mode);
//    }
//
//    @Override public int getTargetPositionTolerance()
//    {
//        return this.targetPositionTolerance;
//    }
//
//    @Override synchronized public void setTargetPositionTolerance(int tolerance)
//    {
//        this.targetPositionTolerance = tolerance;
//    }
//
//    @Override protected void internalSetTargetPosition(int position)
//    {
//        this.controllerEx.setMotorTargetPosition(portNumber, position, this.targetPositionTolerance);
//    }
//
//    @Override public double getCurrent(CurrentUnit unit)
//    {
//        return this.controllerEx.getMotorCurrent(portNumber, unit);
//    }
//
//    @Override
//    public double getCurrentAlert(CurrentUnit unit)
//    {
//        return this.controllerEx.getMotorCurrentAlert(portNumber, unit);
//    }
//
//    @Override
//    public void setCurrentAlert(double current, CurrentUnit unit)
//    {
//        this.controllerEx.setMotorCurrentAlert(portNumber, current, unit);
//    }
//
//    @Override
//    public boolean isOverCurrent()
//    {
//        return this.controllerEx.isMotorOverCurrent(portNumber);
//    }
}
