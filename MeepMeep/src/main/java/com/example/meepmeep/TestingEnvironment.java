package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class TestingEnvironment {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(35, 25, Math.PI / 1.5, Math.PI / 1.5, 14.536)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-37, -62, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-38, -41, Math.toRadians(150)))
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