package com.smartcluster.oracleftc.autonomous.localization;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.smartcluster.oracleftc.hardware.wrappers.Encoder;
import com.smartcluster.oracleftc.math.Pose2d;
import com.smartcluster.oracleftc.math.Pose2dDual;
import com.smartcluster.oracleftc.math.Time;

public class MecanumDriveLocalizer implements Localizer {

    private Encoder frontRight, backRight, frontLeft, backLeft;
    public MecanumDriveLocalizer(Encoder frontRight, Encoder backRight, Encoder frontLeft, Encoder backLeft)
    {
        this.frontRight=frontRight;
        this.backRight=backRight;
        this.frontLeft=frontLeft;
        this.backLeft=backLeft;
    }

    @Override
    public void setPose(Pose2d pose) {

    }

    @Override
    public Pose2dDual<Time> getPose() {
        return null;
    }

    @Override
    public void update() {

    }

}
