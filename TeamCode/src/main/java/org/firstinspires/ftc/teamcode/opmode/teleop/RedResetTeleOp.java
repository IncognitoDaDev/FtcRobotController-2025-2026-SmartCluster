package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "TeleOp")
public class RedResetTeleOp extends BaseTeleOp{
    {

        endPose = new Pose2d(61.25, -61.25, Math.toRadians(0));
        closeShoot = new Pose2d(12,12,Math.toRadians(-135));
        farShoot = new Pose2d(15, -56,Math.toRadians(-120));
        cornerCoordinate = new Pose2d(60,63, Math.toRadians(-45));
        resetEncoder=true;
    }
}
