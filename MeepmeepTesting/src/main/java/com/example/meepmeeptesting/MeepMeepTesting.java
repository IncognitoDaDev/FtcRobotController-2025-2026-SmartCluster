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
        final Pose2d startPose = new Pose2d(13,-62, Math.toRadians(-90));
        final Pose2d shootPose = new Pose2d(15,-56,Math.toRadians(-120));
        final Vector2d shootPoseV = new Vector2d(15, -56);
        final Pose2d stack1 = new Pose2d(25,-35.5,Math.toRadians(0));
        final Pose2d stack2 = new Pose2d(28,-10.5,Math.toRadians(0));
        final Pose2d stack3 = new Pose2d(28,12.5,Math.toRadians(0));
        final Pose2d gate = new Pose2d(55.6,-10 , Math.toRadians(10));
        final Pose2d GateInt = new Pose2d();
        final Pose2d endPose = new Pose2d(24, -15, Math.toRadians(-90));
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(420/25.4, 432/25.4)
                .setStartPose(startPose)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), java.lang.Math.toRadians(180),   15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(shootPose, Math.toRadians(70))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(stack1,Math.toRadians(0))
                        .setTangent(Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(60, -35.5), Math.toRadians(0))
                        .setTangent(Math.toRadians(-120))
                        .splineToConstantHeading(shootPoseV ,Math.toRadians(180))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(stack2,Math.toRadians(0))
                        .setTangent(Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d( 60,-10.5), Math.toRadians(0))
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(shootPoseV,Math.toRadians(180))
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(gate , Math.toRadians(-0))
                        .build());

        Image img = null;
        try { img = ImageIO.read(new File("J:/chestii/imagine.meepmeep/field-2025-juice-dark.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}