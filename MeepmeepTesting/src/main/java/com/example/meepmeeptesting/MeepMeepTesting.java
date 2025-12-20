//package com.example.meepmeeptesting;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//
//import org.rowlandhall.meepmeep.MeepMeep;
//import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
//import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//
//import java.awt.Image;
//import java.io.File;
//import java.io.IOException;
//
//import javax.imageio.ImageIO;
//
//public class MeepMeepTesting {
//    public static void main(String[] args) {
//        MeepMeep meepMeep = new MeepMeep(800);
//        Pose2d startPose = new Pose2d(-11,-57.5,90);
//        Pose2d firstStack = new Pose2d(-25,-40,180);
//        Pose2d secondStack = new Pose2d(-25,-30,180);
//        Pose2d thirdStack = new Pose2d(-25,-20,180);
//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//
//
//            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
//            .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)
//                    .splineToLinearHeading(ShootPose, Math.toRadians(180))
//                    .splineTo(firstStack.vec(),Math.toRadians(180))
//
//            .build());
//
//        Image img = null;
//        try { img = ImageIO.read(new File("C://Users//minec//Documents//field-2025-juice-dark.jpg/")); }
//        catch(IOException e) {}
//
//        meepMeep.setBackground(img)
//            .setDarkMode(true)
//            .setBackgroundAlpha(0.95f)
//            .addEntity(myBot)
//            .start();
//    }
//}