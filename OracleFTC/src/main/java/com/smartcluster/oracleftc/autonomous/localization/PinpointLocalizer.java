package com.smartcluster.oracleftc.autonomous.localization;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.smartcluster.oracleftc.hardware.GoBildaPinpoint;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Pose2d;
import com.smartcluster.oracleftc.math.Pose2dDual;
import com.smartcluster.oracleftc.math.Time;

public class PinpointLocalizer implements Localizer {

    private final GoBildaPinpoint pinpoint;
    public PinpointLocalizer(GoBildaPinpoint pinpoint,double xOffset, double yOffset, double yawScalar)
    {
        this.pinpoint=pinpoint;
        pinpoint.setOffsets(xOffset,yOffset);
        pinpoint.setYawScalar(yawScalar);
        
    }

    @Override
    public void setPose(Pose2d pose) {
        pinpoint.setPosition(pose);
    }

    @Override
    public Pose2dDual<Time> getPose() {
        return new Pose2dDual<Time>(
                new DualNum<Time>(pinpoint.getPosX(), pinpoint.getVelX()),
                new DualNum<Time>(pinpoint.getPosY(), pinpoint.getVelY()),
                new DualNum<Time>(pinpoint.getHeading(), pinpoint.getHeadingVelocity())
        );
    }

    @Override
    public void update() {
        pinpoint.update();
    }

//    @Override
//    public void tune(OpMode opmode) {
//
//    }
}
