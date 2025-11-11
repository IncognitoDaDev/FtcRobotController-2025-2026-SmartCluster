package com.smartcluster.oracleftc.math.control;

import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;

public class MotorFeedforward {
    public double kS;
    public double kV;
    public double kA;

    public MotorFeedforward(double kS, double kV, double kA)
    {
        this.kS=kS;
        this.kV=kV;
        this.kA=kA;
    }

    public double update(DualNum<Time> velocity)
    {
        return kS*Math.signum(velocity.get(0))+kV*velocity.get(0)+kA*velocity.get(1);
    }

    public double update(double velocity, double acceleration)
    {
        return kS*Math.signum(velocity)+kV*velocity+kA*acceleration;
    }

}
