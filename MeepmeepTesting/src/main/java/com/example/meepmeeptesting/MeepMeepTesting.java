package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        final Pose2d shootPose = new Pose2d(12, 12,Math.toRadians(-135));
         final Pose2d stack1 = new Pose2d(28,-35,Math.toRadians(0));
         final Pose2d stack2 = new Pose2d(28,-10.5,Math.toRadians(0));
         final Pose2d stack3 = new Pose2d(28,12,Math.toRadians(0));
         final Pose2d gatePose = new Pose2d(54, -1, Math.toRadians(-90));
         final Pose2d endPose = new Pose2d(24, -15, Math.toRadians(-90));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)


                
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(45.2, 57.7, Math.toRadians(-143)))
                    .setTangent(Math.toRadians(-135))
                    .splineToSplineHeading(shootPose, Math.toRadians(-135))
                    .setTangent(Math.toRadians(-120))
                    .splineToSplineHeading(stack2,Math.toRadians(0))

            .build());

        Image img = null;
        try { img = ImageIO.read(new File("C:/Users/minec/Documents/field-2025-juice-dark.jpg")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
    }
}