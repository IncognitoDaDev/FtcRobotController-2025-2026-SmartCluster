package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
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

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.subsystem.Storage.ArtifactColor;

public class Limelight extends Subsystem {

    private final Limelight3A limelight;
    private ArtifactColor[] order = {ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN};
    private Pose2d limelightPose = new Pose2d(0,0,0);
    public Limelight(OpMode opMode) {
        super(opMode);
        limelight = hardwareMap.get(Limelight3A.class, "Webcam Turret");
        limelight.setPollRateHz(100);
    }

    boolean isFinished = false;

    public ArtifactColor[] getOrder()
    {
        return order;
    }


    ElapsedTime timer = new ElapsedTime();
    public Command scanOrder()
    {
        return new SequentialCommand(
                Command.builder()
                        .init(() ->
                        {
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

                                                isFinished = true;
                                                break;
                                            case 22:
                                                order = new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE};
                                                isFinished = true;
                                                break;
                                            case 23:
                                                order = new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN};

                                                isFinished = true;
                                                break;


                                        }
                                    }
                                }
                            }
                        })
                        .finished(() -> timer.milliseconds() > 400 || isFinished)
                        .build()
        );

    }
//    public Command getPose(boolean color,Localizer localizer){
//        return Command.builder()
//                .init(()->{
//                    timer.reset();
//                })
//                .update(()->{LLResult result = limelight.getLatestResult();
//        if (result != null && result.isValid())
//        {
//            localizer.update();
//            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//            limelight.updateRobotOrientation(localizer.getPose().heading.log());
//            if (!fiducials.isEmpty())
//            {
//                for (LLResultTypes.FiducialResult tag : fiducials) {
//                    switch (tag.getFiducialId()) {
//                        case 20:
//                            if(color)limelightPose = new Pose2d(result.getBotpose_MT2().getPosition().x,result.getBotpose_MT2().getPosition().y,result.getBotpose_MT2().getOrientation().getYaw(AngleUnit.RADIANS));
//                            double x = result.getBotpose_MT2().getPosition().x;
//                            double y = result.getBotpose_MT2().getPosition().y;
//                            telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
//                            break;
//                        case 24:
//                            if(!color)limelightPose = new Pose2d(result.getBotpose().getPosition().x,result.getBotpose().getPosition().y,result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS));
//                            double X = result.getBotpose_MT2().getPosition().x;
//                            double Y = result.getBotpose_MT2().getPosition().y;
//                            telemetry.addData("MT2 Location:", "(" + X + ", " + Y + ")");
//                    }
//                }
//            }
//        }
//                })
//                .build();
//
//    }

//    public Pose2d avgPose(Pose2d a, Pose2d b){
//        telemetry.addData("Limelight Pose estimate",limelightPose);
//        return new Pose2d((a.position.x+b.position.x)/2,(a.position.y+b.position.y)/2,Math.toRadians((a.heading.log()+b.heading.log())/2));
//    }
//    public Pose2d getPose(){
//        telemetry.addData("Limelight Pose estimate",limelightPose);
//        return limelightPose;
//    }

//    public Pose2d getPose(){
//        telemetry.addData("Limelight Pose estimate",limelightPose);
//        return limelightPose;
//    }


    private double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }


    public Command updatePoseFromTags(Localizer localizer) {
        return Command.builder()
                .update(() -> {
                    LLResult result = limelight.getLatestResult();
                    if (result == null || !result.isValid()) return;
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    if (fiducials.isEmpty()) return;
                    Pose2d llPose = new Pose2d(
                            result.getBotpose_MT2().getPosition().x,
                            result.getBotpose_MT2().getPosition().y,
                            normalizeAngle(
                                    result.getBotpose_MT2()
                                            .getOrientation()
                                            .getYaw(AngleUnit.RADIANS) + Math.PI
                            )
                    );
                    limelightPose = llPose;
                    localizer.setPose(llPose);
                })
                .build();
    }

    public void reset() {
       limelight.start();
    }
}