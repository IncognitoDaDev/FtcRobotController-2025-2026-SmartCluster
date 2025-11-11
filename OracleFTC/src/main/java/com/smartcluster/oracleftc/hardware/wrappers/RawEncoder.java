package com.smartcluster.oracleftc.hardware.wrappers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;

public class RawEncoder implements Encoder {
    private final Encoder encoder;

    public RawEncoder(Encoder encoder)
    {
        this.encoder=encoder;
    }
    public RawEncoder(DcMotorEx motor)
    {
        this.encoder=new Encoder() {
            private int lastPosition =  0;
            private Direction direction;
            private double applyDirection(double x)
            {
                if(motor.getDirection()==DcMotorSimple.Direction.REVERSE)
                    x=-x;
                if(direction==Direction.REVERSE)
                    x=-x;
                return x;
            }

            @Override
            public void setDirection(Direction direction) {
                this.direction=direction;
            }

            @Override
            public Direction getDirection() {
                return direction;
            }

            @Override
            public DualNum<Time> getCurrentPosition() {
                return new DualNum<>(applyDirection(motor.getCurrentPosition()-lastPosition), applyDirection(motor.getVelocity()));
            }

            @Override
            public void reset() {
                lastPosition= motor.getCurrentPosition();
            }

            @Override
            public void reset(double position) {
                lastPosition= (int) (motor.getCurrentPosition()-position);
            }
        };
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
        return encoder.getCurrentPosition();
    }


    @Override
    public void reset() {
        encoder.reset();
    }

    @Override
    public void reset(double position) {
        encoder.reset(position);
    }


}
