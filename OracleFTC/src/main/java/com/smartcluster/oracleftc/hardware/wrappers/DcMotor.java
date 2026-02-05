package com.smartcluster.oracleftc.hardware.wrappers;

import com.smartcluster.oracleftc.hardware.OmegaPowerCollector;

public interface DcMotor {

    int portNumber = 0;
    boolean isOnControlHub = false;

    OmegaPowerCollector powerCollector = null;

    void setDestination(OmegaPowerCollector powerCollector, boolean isPluggedIntoControlHub);

    void setPower(double power);
    void setPowerValue(double power);
}
