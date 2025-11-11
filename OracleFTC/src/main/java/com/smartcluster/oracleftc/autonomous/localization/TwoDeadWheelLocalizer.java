package com.smartcluster.oracleftc.autonomous.localization;

import com.smartcluster.oracleftc.hardware.wrappers.Encoder;
import com.smartcluster.oracleftc.math.Pose2d;

public abstract class TwoDeadWheelLocalizer implements Localizer
{
    private final Encoder parallelEncoder, perpendicularEncoder;
    private final Pose2d parallelEncoderPose, perpendicularEncoderPose;

    protected TwoDeadWheelLocalizer(Encoder parallelEncoder, Encoder perpendicularEncoder, Pose2d parallelEncoderPose, Pose2d perpendicularEncoderPose)
    {
        this.parallelEncoder=parallelEncoder;
        this.perpendicularEncoder=perpendicularEncoder;

        this.parallelEncoderPose=parallelEncoderPose;
        this.perpendicularEncoderPose=perpendicularEncoderPose;
    }




}
