package com.smartcluster.oracleftc.hardware.wrappers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.filters.MovingMedianFilter;

@SuppressWarnings({"unused"})
public class OverflowEncoder implements Encoder {
    private final Encoder encoder;
    private final ElapsedTime lastUpdate = new ElapsedTime();
    private final MovingMedianFilter medianFilter = new MovingMedianFilter(3);
    private Direction direction;
    private int lastPosition;

    public OverflowEncoder(Encoder encoder) {
        this.encoder = encoder;
        this.direction = Direction.FORWARD;
        lastPosition = (int) getCurrentPosition().get(0);
    }

    public OverflowEncoder(DcMotorEx motor) {
        this(new RawEncoder(motor));
    }

    @Override
    public void setDirection(Direction direction) {
        encoder.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return encoder.getDirection();
    }

    @Override
    public DualNum<Time> getCurrentPosition() {
        DualNum<Time> encoderPosition = encoder.getCurrentPosition();
        double dt = lastUpdate.seconds();
        double v = medianFilter.update((encoderPosition.get(0) - this.lastPosition) / dt);
        lastPosition = (int) encoderPosition.get(0);
        lastUpdate.reset();
        return new DualNum<>(encoderPosition.get(0),inverseOverflow((int) encoderPosition.get(1), v));
    }

    @Override
    public void reset() {
        encoder.reset();
        medianFilter.reset();
        lastUpdate.reset();
        lastPosition=0;
    }

    @Override
    public void reset(double position) {
        encoder.reset(position);
        medianFilter.reset();
        lastUpdate.reset();
        lastPosition= (int) position;
    }

    private static final int CPS_STEP = 65536;

    private static int inverseOverflow(int input, double estimate) {
        int real = input & '\uffff';
        real += real % 20 / 4 * CPS_STEP;
        real += (int) Math.rint((estimate - (double) real) / (double) 327680) * 5 * CPS_STEP;
        return real;
    }
}
