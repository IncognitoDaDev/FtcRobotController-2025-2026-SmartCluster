package com.smartcluster.oracleftc.math.control;

import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Time;

public abstract class MotionProfile {
    abstract DualNum<Time> getMotionState(double distance, double time);
}
