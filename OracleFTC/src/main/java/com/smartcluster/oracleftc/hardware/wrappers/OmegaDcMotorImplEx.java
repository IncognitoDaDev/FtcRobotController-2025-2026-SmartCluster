package com.smartcluster.oracleftc.hardware.wrappers;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.smartcluster.oracleftc.hardware.OmegaPowerCollector;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

public class OmegaDcMotorImplEx implements DcMotor{

    private int portNumber = 0;
    private boolean isOnControlHub = false;
    private OmegaPowerCollector powerCollector = null;
    private DcMotorImplEx DcMotor;

    public OmegaDcMotorImplEx(DcMotorImplEx DcMotor)
    {
        this.DcMotor = DcMotor;
        portNumber = DcMotor.getPortNumber();
    }

    public OmegaDcMotorImplEx(DcMotorImplEx DcMotor, OmegaPowerCollector powerCollector, boolean isPluggedIntoControlHub)
    {
        this.DcMotor = DcMotor;
        portNumber = DcMotor.getPortNumber();
        this.powerCollector = powerCollector;
        isOnControlHub = isPluggedIntoControlHub;
    }

    public DcMotorImplEx getDcMotor() {return DcMotor;}

    @Override
    public void setDestination(OmegaPowerCollector powerCollector, boolean isPluggedIntoControlHub) {
        this.powerCollector = powerCollector;
        isOnControlHub = isPluggedIntoControlHub;
    }

    @Override
    public void setPower(double power) {
        if (DcMotor.getMode() == com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION) {
            power = Math.abs(power);
        } else {
            power = adjustPower(power);
        }
    }

    @Override
    public void setPowerValue(double power) {
        powerCollector.bulkValues.setDcMotorValue(portNumber + (isOnControlHub ? 0 : 4), power);
    }

    double adjustPower(double power) {
        if (getOperationalDirection() == DcMotorSimple.Direction.REVERSE) power = -power;
        return power;
    }

    protected DcMotorSimple.Direction getOperationalDirection() {
        return DcMotor.getMotorType().getOrientation() == Rotation.CCW ? DcMotor.getDirection().inverted() : DcMotor.getDirection();
    }
}
