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

public class RedBackdrop {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(62, 14, 0))
                                .strafeTo(new Vector2d(58, 14))
                                .strafeTo(new Vector2d(45, 35))
                                .turn(Math.PI / 2.0)
                                .strafeTo(new Vector2d(27.5, 35))
                                .strafeTo(new Vector2d(27.25, 44.25))
                                .strafeTo(new Vector2d(27.5, 40))
                                .strafeTo(new Vector2d(34, 18))
                                .strafeTo(new Vector2d(34, 0))
                                .strafeTo(new Vector2d(34, 18))
                                .strafeTo(new Vector2d(8, 53))
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