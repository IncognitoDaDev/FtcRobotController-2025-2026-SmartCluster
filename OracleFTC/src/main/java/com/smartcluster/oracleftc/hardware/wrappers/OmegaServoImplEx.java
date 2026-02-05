package com.smartcluster.oracleftc.hardware.wrappers;

import static com.qualcomm.robotcore.hardware.Servo.MAX_POSITION;
import static com.qualcomm.robotcore.hardware.Servo.MIN_POSITION;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import com.smartcluster.oracleftc.hardware.OmegaPowerCollector;

public class OmegaServoImplEx implements Servo {

    private int portNumber = 0;
    private boolean isOnControlHub = false;
    private OmegaPowerCollector powerCollector = null;
    private ServoImplEx servo;
    protected double          limitPositionMin = MIN_POSITION;
    protected double          limitPositionMax = MAX_POSITION;

    public ServoImplEx getServo() {return servo;}

    public OmegaServoImplEx(ServoImplEx servo)
    {
        this.servo = servo;
        portNumber = servo.getPortNumber();
    }

    public OmegaServoImplEx(ServoImplEx servo, OmegaPowerCollector powerCollector, boolean isPluggedIntoControlHub)
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
    public void setPosition(double position) {
        position = Range.clip(position, MIN_POSITION, MAX_POSITION);
        if (servo.getDirection() == com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE) position = reverse(position);
        double scaled = Range.scale(position, MIN_POSITION, MAX_POSITION, limitPositionMin, limitPositionMax);
        setPositionValue(position);
    }

    @Override
    public void setPositionValue(double power) {
        powerCollector.bulkValues.setServoValue(portNumber + (isOnControlHub ? 0 : 4), power);
    }

    private double reverse(double position) {
        return MAX_POSITION - position + MIN_POSITION;
    }

}
