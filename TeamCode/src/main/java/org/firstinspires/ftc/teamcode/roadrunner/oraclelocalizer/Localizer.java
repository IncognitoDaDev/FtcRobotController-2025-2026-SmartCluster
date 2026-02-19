package org.firstinspires.ftc.teamcode.roadrunner.oraclelocalizer;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.smartcluster.oracleftc.math.DualNum;
import com.smartcluster.oracleftc.math.Pose2dDual;
import com.smartcluster.oracleftc.math.PoseVelocity2d;
import com.smartcluster.oracleftc.math.Rotation2dDual;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.Twist2dDual;
import com.smartcluster.oracleftc.math.Vector2d;
import com.smartcluster.oracleftc.math.Vector2dDual;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Localizer  {


    public Localizer(HardwareMap hardwareMap, Telemetry telemetry) {
        pose=new Pose2dDual<>(new Vector2dDual<>(new DualNum<>(0),new DualNum<>(0)), Rotation2dDual.exp(new DualNum<>(0)));
    }

    protected Pose2dDual<Time> pose;

    public abstract Twist2dDual<Time> update();

    public Pose2dDual<Time> getPose() {
        return pose;
    }

    public void setPose(Pose2dDual<Time> pose)
    {
        this.pose=pose;
    }

    public void setPose(Pose2d pose)
    {
        com.smartcluster.oracleftc.math.Pose2d convertedPose =
                new com.smartcluster.oracleftc.math.Pose2d(pose.position.x, pose.position.y, pose.heading.log());
        this.pose = new Pose2dDual<>(convertedPose, new PoseVelocity2d(new Vector2d(0,0), 0));
    }

    public void setPose(com.smartcluster.oracleftc.math.Pose2d pose)
    {
        this.pose = new Pose2dDual<>(pose, new PoseVelocity2d(new Vector2d(0,0), 0));
    }
}
