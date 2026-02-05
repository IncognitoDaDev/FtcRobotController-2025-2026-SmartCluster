package com.smartcluster.oracleftc.hardware.wrappers;

import static com.qualcomm.robotcore.hardware.Servo.MAX_POSITION;
import static com.qualcomm.robotcore.hardware.Servo.MIN_POSITION;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import com.smartcluster.oracleftc.hardware.OmegaPowerCollector;

public class OmegaCRServoImplEx implements CRServo {

    private int portNumber = 0;
    private boolean isOnControlHub = false;
    private OmegaPowerCollector powerCollector = null;
    private CRServoImplEx servo;
    protected static final double apiPowerMin = -1.0;
    protected static final double apiPowerMax =  1.0;
    protected static final double apiServoPositionMin = 0.0;
    protected static final double apiServoPositionMax = 1.0;

    public CRServoImplEx getServo() {return servo;}

    public OmegaCRServoImplEx(CRServoImplEx servo)
    {
        this.servo = servo;
        portNumber = servo.getPortNumber();
    }

    public OmegaCRServoImplEx(CRServoImplEx servo, OmegaPowerCollector powerCollector, boolean isPluggedIntoControlHub)
    {
        this.servo = servo;
        portNumber = servo.getPortNumber();
        this.powerCollector = powerCollector;
        isOnControlHub = isPluggedIntoControlHub;
    }

    @Override
    public void setDestination(OmegaPowerCollector powerCollector, boolean isPluggedIntoControlHub) {
        this.powerCollector = powerCollector;
        isOnControlHub = isPluggedIntoControlHub;
    }

    @Override
    public void setPower(double power) {
        if (servo.getDirection() == DcMotorSimple.Direction.REVERSE) power = -power;
        power = Range.clip(power, apiPowerMin, apiPowerMax);
        power = Range.scale(power, apiPowerMin, apiPowerMax, apiServoPositionMin, apiServoPositionMax);
        setPowerValue(power);
    }

    @Override
    public void setPowerValue(double power) {
        powerCollector.bulkValues.setServoValue(portNumber + (isOnControlHub ? 0 : 4), power);
    }
}
