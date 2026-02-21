package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.InstantCommand;
import com.smartcluster.oracleftc.commands.SequentialCommand;
import com.smartcluster.oracleftc.commands.WaitCommand;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;
import com.smartcluster.oracleftc.math.Pose2d;
import com.smartcluster.oracleftc.math.Pose2dDual;
import com.smartcluster.oracleftc.math.PoseVelocity2d;
import com.smartcluster.oracleftc.math.Time;
import com.smartcluster.oracleftc.math.Vector2d;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.roadrunner.oraclelocalizer.SmartLocalizer;
import org.firstinspires.ftc.teamcode.subsystem.Storage.ArtifactColor;

public class Limelight extends Subsystem {

    private final Limelight3A limelight;
    public static ArtifactColor[] order = {ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN};
    private
    int orderInt = -1;

    private SmartLocalizer localizer;
    static public int limelightRejectionThreshold = 4;

    public Limelight(OpMode opMode, SmartLocalizer localizer) {
        super(opMode);
        limelight = hardwareMap.get(Limelight3A.class, "Webcam Turret");
        limelight.setPollRateHz(100);
        this.localizer = localizer;
    }

    boolean isFinished = false;

    public ArtifactColor[] getOrder()
    {
        return order;
    }


    public Command scanOrder()
    {
        ElapsedTime timer = new ElapsedTime();

        return Command.builder()
                        .init(() ->
                        {
                            limelight.pipelineSwitch(0);
                            limelight.start();
                            isFinished = false;
                            timer.reset();
                        })
                        .update(() ->
                        {
                            LLResult result = limelight.getLatestResult();
                            if (result != null && result.isValid())
                            {
                                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                                if (!fiducials.isEmpty())
                                {
                                    for (LLResultTypes.FiducialResult tag : fiducials) {

                                        switch (tag.getFiducialId()) {
                                            case 21:
                                                order = new ArtifactColor[]{ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
                                                orderInt = 0;
                                                isFinished = true;
                                                break;
                                            case 22:
                                                order = new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE};
                                                orderInt = 1;
                                                isFinished = true;
                                                break;
                                            case 23:
                                                order = new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN};
                                                orderInt = 2;
                                                isFinished = true;
                                                break;
                                        }

                                    }
                                }
                            }
                        })
                        .finished(() -> timer.milliseconds() > 500 || isFinished)
                        .build();
    }

    public String getOrderString()
    {
        switch(orderInt)
        {
            case -1: return "none";
            case 0: return "G-P-P";
            case 1: return "P-G-P";
            case 2: return "P-P-G";
        }

        return "How did we get here?";
    }

//    private boolean isValidPose(Pose2dDual<Time> newPose)
//    {
//        return !(Double.isNaN(newPose.heading.log().get(0)) ||
//                Double.isNaN(newPose.position.x.get(0)) ||
//                Double.isNaN(newPose.position.y.get(0)) ||
//                localizer.getPose().position.minus(newPose.position).sqrNorm().get(0) > limelightRejectionThreshold*limelightRejectionThreshold);
//    }
//
//    public Command changePipeline(int pipeline)
//    {
//        return new SequentialCommand(
//                new InstantCommand(() -> limelight.pipelineSwitch(pipeline)),
//                new WaitCommand(20)
//        );
//    }
//
//    public void getEstimatedPose_MT2()
//    {
//        LLResult scan = limelight.getLatestResult();
//
//        double robotYaw = Math.toDegrees(localizer.getPose().heading.log().get(0));
//        robotYaw = (robotYaw >= -90 ? robotYaw: -robotYaw);
//        limelight.updateRobotOrientation(robotYaw);
//
//        if (scan != null && scan.isValid())
//        {
//            Pose3D botpose = scan.getBotpose_MT2();
//            Pose2d botposeTranslated = new Pose2d(-botpose.getPosition().toUnit(DistanceUnit.INCH).x, botpose.getPosition().toUnit(DistanceUnit.INCH).y, localizer.getPose().heading.log().get(0));
//            Pose2dDual<Time> newPose = new Pose2dDual<>(botposeTranslated, new PoseVelocity2d(new Vector2d(0,0), 0));
//
//            double x = botposeTranslated.position.x;
//            double y = botposeTranslated.position.y;
//
//            telemetry.addData("Camera X", x);
//            telemetry.addData("Camera Y", y);
//            telemetry.addData("Excepted angle", scan.getBotpose().getOrientation().getYaw());
//            telemetry.addData("Instead got angle", robotYaw);
//
//
//
//            if (isValidPose(newPose)) {
//                telemetry.addLine("Good pose!");
//                localizer.setPosition(newPose);
//            }
//        }
//    }

    public void reset() {
       limelight.start();
    }
}