package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "TeleOp")
public class BlueTeleOp extends BaseTeleOp{
    {
        cornerCoordinates=new Pose2d(-60,63,-45);
        endPose = new Pose2d(-24, -15, Math.toRadians(0));

    }
}
