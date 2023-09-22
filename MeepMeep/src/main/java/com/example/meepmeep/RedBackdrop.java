package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class RedBackdrop {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(62, 12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(35.5, 49.5, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(18, 31, Math.toRadians(310)))
                                .lineToSplineHeading(new Pose2d(11.5, -60, Math.toRadians(270)))
                                .build()
                );

        Image img = null;
        try {
            img = ImageIO.read(new File("/Users/awsare/Documents/CENTERSTAGE/MeepMeep/src/main/java/com/example/meepmeep/Juice-CENTERSTAGE-Dark.png"));
        } catch (IOException e) {

        }

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}