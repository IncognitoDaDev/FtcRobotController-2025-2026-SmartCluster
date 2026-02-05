package com.smartcluster.oracleftc.hardware.wrappers;

import com.smartcluster.oracleftc.hardware.OmegaPowerCollector;

public interface Servo {

    void setDestination(OmegaPowerCollector powerCollector, boolean isPluggedIntoControlHub);
    void setPosition(double power);
    void setPositionValue(double power);
}
