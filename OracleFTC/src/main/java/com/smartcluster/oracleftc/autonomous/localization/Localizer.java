package com.smartcluster.oracleftc.autonomous.localization;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.smartcluster.oracleftc.math.Pose2d;
import com.smartcluster.oracleftc.math.Pose2dDual;
import com.smartcluster.oracleftc.math.Time;

public interface Localizer {
    void setPose(Pose2d pose);
    Pose2dDual<Time> getPose();
    void update();
}
