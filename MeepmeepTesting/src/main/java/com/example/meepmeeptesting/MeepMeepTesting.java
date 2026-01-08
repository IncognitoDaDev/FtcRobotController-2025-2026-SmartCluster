package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
         final Pose2d startPose = new Pose2d(-61.25, -61.25, Math.toRadians(0));
         final Pose2d shootPose = new Pose2d(-12, 12,Math.toRadians(-45));
         final Pose2d stack1 = new Pose2d(-28,-35,Math.toRadians(180));
         final Pose2d stack2 = new Pose2d(-28,-10.5,Math.toRadians(180));
         final Pose2d stack3 = new Pose2d(-28,12,Math.toRadians(180));
         final Pose2d endPose = new Pose2d(-24, -15, Math.toRadians(-90));
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d StartPose = new Pose2d(-59, -34, Math.toRadians(90));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(420/25.4, 432/25.4)
                .setStartPose(StartPose)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)
//                        .setTangent(Math.toRadians(-45))
//                        .splineToLinearHeading(shootPose, Math.toRadians(-45))
//                        .setTangent(Math.toRadians(0))
//                        .splineToLinearHeading(stack3,Math.toRadians(180))
//                        .setTangent(Math.toRadians(180))
//                        .splineToConstantHeading(new Vector2d(-56, 12), Math.toRadians(180))
//                        .setTangent(Math.toRadians(60))
//                        .splineToLinearHeading(shootPose, Math.toRadians(60))
//                        .splineToLinearHeading(stack2,Math.toRadians(135))
//                        .setTangent(Math.toRadians(0))
//                        .splineToConstantHeading(new Vector2d(-63, -10.5), Math.toRadians(0))
//                        .setTangent(Math.toRadians(-120))
//                        .splineToLinearHeading(shootPose, Math.toRadians(-140))
                        .setTangent(Math.toRadians(45))
                        .splineToLinearHeading(endPose,Math.toRadians(45))
                        .build());

        Image img = null;
        try { img = ImageIO.read(new File("./MeepMeepTesting/field-2025-juice-dark.png/")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}