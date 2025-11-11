package com.smartcluster.oracleftc.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.smartcluster.oracleftc.autonomous.follower.PIDToPointFollower;
import com.smartcluster.oracleftc.autonomous.localization.Localizer;
import com.smartcluster.oracleftc.autonomous.localization.MecanumDriveLocalizer;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.math.Pose2d;
import com.smartcluster.oracleftc.math.Pose2dDual;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.control.MecanumKinematics;
import com.smartcluster.oracleftc.math.control.MotorFeedforward;
import org.jetbrains.annotations.Nullable;

public abstract class MecanumDrive extends Subsystem {

    /**
     * Localizer used by this mecanum drive
     */
    protected Localizer localizer;

    /**
     * Motor feedforward for the drivetrain motors
     */
    protected MotorFeedforward feedforward;

    /**
     * Kinematics for the mecanum drive
     */
    protected MecanumKinematics kinematics;

    public PIDToPointFollower p2p;
//    public PurePursuitFollower pp;
//    public RoadRunnerFollower rr;
//    public GVFFollower gvf;
//    public UnprofiledBezierFollower ubf;

    public MecanumDrive(OpMode opMode) {
        super(opMode);
    }

    public Pose2dDual<Time> getPose()
    {
        return localizer.getPose();
    }

    public void update()
    {
        localizer.update();
    }

    public final void setMotorPowers(double[] motorPowers)
    {
        setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
    }

    public abstract void setMotorPowers(double frontRightPower, double backRightPower, double frontLeftPower, double backLeftPower);

    public abstract boolean defaultEndCondition(Pose2d target);

}
