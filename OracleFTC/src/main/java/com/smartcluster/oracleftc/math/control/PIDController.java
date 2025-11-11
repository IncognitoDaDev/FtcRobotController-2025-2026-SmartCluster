package com.smartcluster.oracleftc.math.control;

import com.qualcomm.robotcore.robot.RobotState;
import com.smartcluster.oracleftc.math.filters.LowPassFilter;
import java.lang.reflect.Field;
import java.util.function.Supplier;

public class PIDController implements Cloneable{

    public double p;
    public double i;
    public double d;

    private long lastTimestamp=0;
    private double integral = 0;
    private double lastError = 0;
    private final LowPassFilter derivativeFilter;
    public PIDController(double p, double i, double d)
    {
        this.p=p;
        this.i=i;
        this.d=d;
        derivativeFilter=new LowPassFilter(0);
    }

    public PIDController(double p, double i, double d, double lowPassGain)
    {
        this.p=p;
        this.i=i;
        this.d=d;
        derivativeFilter=new LowPassFilter(lowPassGain);
    }

    public double update(double target, double value) {
        double error = target - value;
        long timestamp = System.nanoTime();
        double derivative = 0;
        if (lastTimestamp != 0) {
            double deltaTime = (timestamp - lastTimestamp) / 1E9;
            integral += deltaTime * error;
            if(Double.isNaN(integral))
            {
                integral=0;
            }
            if (deltaTime > 1E-6) derivative = derivativeFilter.update((error - lastError) / deltaTime);

        }
        lastError = error;
        lastTimestamp = timestamp;
        return error * p + integral * i + derivative * d;
    }
    public double update(double target, double value, double derivative) {
        double error = target - value;
        long timestamp = System.nanoTime();

        if (lastTimestamp != 0) {
            double deltaTime = (timestamp - lastTimestamp) / 1E9;
            if (deltaTime > 1E-6) derivative = derivativeFilter.update(derivative / deltaTime);
        }
        lastError = error;
        lastTimestamp = timestamp;
        return error * p + integral * i + derivative * d;
    }

    public void reset()
    {
        lastTimestamp=0;
        integral=0;
        lastError=0;
        derivativeFilter.reset();
    }


    @Override
    public PIDController clone() {
        try {
            PIDController clone = (PIDController) super.clone();
            // TODO: copy mutable state here, so the clone can't change the internals of the original
            clone.p=p;
            clone.i=i;
            clone.d=d;
            return clone;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }
}
