package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.vision.PropCamera;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
@Autonomous
public class FullBlueBackdrop extends LinearOpMode {

    MecanumDrive drive;
    Robot robot;
    PropCamera camera;

    int randomization = 2;

    public static double HEADING = 5;

    Pose2d startPose = new Pose2d(14, 62, -Math.PI / 2.0);

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);

        robot = new Robot();
        robot.init(hardwareMap, false);

        camera = new PropCamera(hardwareMap, telemetry, "Blue");

        Action act1 = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(14, 58))
                .strafeToConstantHeading(new Vector2d(35, 45))
                .turnTo(Math.toRadians(HEADING))
                .build();

        Action goToYellowA = drive.actionBuilder(new Pose2d(35, 45, 0))
                .strafeToConstantHeading(new Vector2d(35, 42.5))
                .strafeToConstantHeading(new Vector2d(44.25, 42.5))
                .build();

        Action backYellowA = drive.actionBuilder(new Pose2d(44.25, 42.5, 0))
                .strafeToConstantHeading(new Vector2d(40, 42.5))
                .build();

        Action goToYellowB = drive.actionBuilder(new Pose2d(35, 45, 0))
                .strafeToConstantHeading(new Vector2d(35, 37))
                .strafeToConstantHeading(new Vector2d(44.25, 37))
                .build();

        Action backYellowB = drive.actionBuilder(new Pose2d(44.25, 37, 0))
                .strafeToConstantHeading(new Vector2d(40, 37))
                .build();

        Action goToYellowC = drive.actionBuilder(new Pose2d(35, 45, 0))
                .strafeToConstantHeading(new Vector2d(35, 31))
                .strafeToConstantHeading(new Vector2d(44.25, 31))
                .build();

        Action backYellowC = drive.actionBuilder(new Pose2d(44.25, 31, 0))
                .strafeToConstantHeading(new Vector2d(40, 31))
                .build();

        Action purpleA = drive.actionBuilder(new Pose2d(40, 42.5, 0))
                .strafeToConstantHeading(new Vector2d(30, 27))
                .strafeToConstantHeading(new Vector2d(23, 27))
                .strafeToConstantHeading(new Vector2d(30, 27))
                .strafeToConstantHeading(new Vector2d(53, 12))
                .build();

        Action purpleB = drive.actionBuilder(new Pose2d(40, 37, 0))
                .strafeToConstantHeading(new Vector2d(20, 24))
                .strafeToConstantHeading(new Vector2d(12, 24))
                .strafeToConstantHeading(new Vector2d(20, 24))
                .strafeToConstantHeading(new Vector2d(53, 12))
                .build();

        Action purpleC = drive.actionBuilder(new Pose2d(40, 31, 0))
                .strafeToConstantHeading(new Vector2d(18, 34))
                .strafeToConstantHeading(new Vector2d(2, 34))
                .strafeToConstantHeading(new Vector2d(18, 34))
                .strafeToConstantHeading(new Vector2d(53, 12))
                .build();

        robot.setClawClosed();
        sleep(2000);
        robot.moveBase(0.7);

        while (opModeInInit()) {
            randomization = camera.getRandomization();
            telemetry.addData("Randomization", randomization);
            telemetry.update();
        }

        waitForStart();

        camera.stopStreaming();

        telemetry.addData("Randomization", randomization);
        telemetry.update();

        Actions.runBlocking(act1);
        robot.setScoring();

        if (randomization == 0) {
            Actions.runBlocking(goToYellowA);
        } else if (randomization == 1) {
            Actions.runBlocking(goToYellowB);
        } else {
            Actions.runBlocking(goToYellowC);
        }

        robot.setScoringLifted();
        robot.setClawScoreOpen();
        sleep(900);

        if (randomization == 0) {
            Actions.runBlocking(backYellowA);
        } else if (randomization == 1) {
            Actions.runBlocking(backYellowB);
        } else {
            Actions.runBlocking(backYellowC);
        }

        robot.setRetracted();

        if (randomization == 0) {
            Actions.runBlocking(purpleA);
        } else if (randomization == 1) {
            Actions.runBlocking(purpleB);
        } else {
            Actions.runBlocking(purpleC);
        }

    }
}
