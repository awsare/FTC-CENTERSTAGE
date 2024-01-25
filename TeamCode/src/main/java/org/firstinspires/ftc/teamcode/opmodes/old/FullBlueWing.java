package org.firstinspires.ftc.teamcode.opmodes.old;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.vision.PropCamera;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
@Disabled
@Autonomous(name = "Blue Wing (Purple) \uD83D\uDC0B", group = "blue")
public class FullBlueWing extends LinearOpMode {

    MecanumDrive drive;
    Robot robot;
    PropCamera camera;

    int randomization = 0;

    Pose2d startPose = new Pose2d(-37, 62, -Math.PI / 2.0);

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);
        robot = new Robot();
        robot.init(hardwareMap, false);

        camera = new PropCamera(hardwareMap, telemetry, "Blue", "Right");

        Action toScoreC = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(-47, 45))
                .build();

        Action toScoreB = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(-41, 41))
                .build();

        Action toScoreA = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(-33, 43))
                .turnTo(Math.toRadians(-30))
                .build();

        robot.setClawClosed();
        sleep(1000);
        robot.moveBase(0.7);

        while (opModeInInit()) {
            randomization = camera.getRandomization();
            telemetry.addData("Randomization", randomization);
            telemetry.update();
        }

        waitForStart();

        if (randomization == 0) {
            Actions.runBlocking(toScoreA);
        } else if (randomization == 1) {
            Actions.runBlocking(toScoreB);
        } else {
            Actions.runBlocking(toScoreC);
        }

        robot.setScoring();
        robot.moveTop(0.7);
        sleep(1500);
        robot.setClawScoreOpen();
        sleep(1000);
        robot.setClawClosed();
        robot.setRetracted();
        sleep(1000);
    }
}
