package org.firstinspires.ftc.teamcode.opmodes.previous;


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
@Autonomous(name = "Adj New Full Blue Backdrop \uD83D\uDC0B", group = "blue")
public class AdjBlueBackdrop extends LinearOpMode {

    MecanumDrive drive;
    Robot robot;
    PropCamera camera;

    int randomization = 0;

    Pose2d startPose = new Pose2d(13, 62, -Math.PI / 2.0);

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);

        robot = new Robot();
        robot.init(hardwareMap, false);

        camera = new PropCamera(hardwareMap, telemetry, "Blue", "Left");

        Action act11 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(10.5, 41), Math.toRadians(-150))
                .build();

        Action act12 = drive.actionBuilder(new Pose2d(10.5, 41, Math.toRadians(-150)))
                .strafeToLinearHeading(new Vector2d(37, 32), 0)
                .build();

        Action act13 = drive.actionBuilder(new Pose2d(37, 32, 0))
                .strafeTo(new Vector2d(45.5, 33.5))
                .build();

        Action act14 = drive.actionBuilder(new Pose2d(45.5, 33.5, 0))
                .strafeTo(new Vector2d(37, 33.5))
                .build();

        Action act15 = drive.actionBuilder(new Pose2d(37, 33.5, 0))
                .strafeTo(new Vector2d(40, 62))
                .build();

        Action act21 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(17, 38))
                .build();

        Action act22 = drive.actionBuilder(new Pose2d(17, 38, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(41, 40), 0)
                .build();

        Action act23 = drive.actionBuilder(new Pose2d(41, 40, 0))
                .strafeTo(new Vector2d(44.5, 40))
                .build();

        Action act24 = drive.actionBuilder(new Pose2d(44.5, 40, 0))
                .strafeTo(new Vector2d(41, 40))
                .build();

        Action act25 = drive.actionBuilder(new Pose2d(41, 40, 0))
                .strafeTo(new Vector2d(42, 62))
                .build();

        Action act31 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(23.5, 48))
                .build();

        Action act32 = drive.actionBuilder(new Pose2d(23.5, 48, Math.toRadians(-90)))
                .strafeTo(new Vector2d(30, 48))
                .strafeToLinearHeading(new Vector2d(37, 45), 0)
                .build();

        // lowkey idk
        Action act33 = drive.actionBuilder(new Pose2d(37, 45, 0))
                .strafeTo(new Vector2d(45.5, 45))
                .build();

        Action act34 = drive.actionBuilder(new Pose2d(45.5, 45, 0))
                .strafeTo(new Vector2d(37, 45))
                .build();

        Action act35 = drive.actionBuilder(new Pose2d(37, 45, 0))
                .strafeTo(new Vector2d(40, 62))
                .build();

        robot.setClawClosed();
        sleep(1000);
        robot.setRetracted();
        robot.moveBase(0.5);

        while (opModeInInit()) {
            randomization = camera.getRandomization();
            telemetry.addData("Randomization", randomization);
            telemetry.update();
        }

        waitForStart();

        camera.stopStreaming();

        telemetry.addData("Randomization", randomization);
        telemetry.update();

        if (randomization == 2) {
            Actions.runBlocking(act11);
        } else if (randomization == 1) {
            Actions.runBlocking(act21);
        } else {
            Actions.runBlocking(act31);
        }

        robot.setIntakeDown();
        robot.moveBase(0.1);
        robot.moveTop(0.45);
        robot.moveWrist(0.275);
        sleep(1500);
        robot.setClawScoreOpen();
        sleep(1000);
        robot.setRetracted();
        robot.setClawClosed();
        robot.moveBase(0.55);
        sleep(1000);

        if (randomization == 2) {
            Actions.runBlocking(act12);
        } else if (randomization == 1) {
            Actions.runBlocking(act22);
        } else {
            Actions.runBlocking(act32);
        }

        robot.setRetracted();
        sleep(1000);
        robot.setClawOpen();
        sleep(250);
        robot.setRetractedLowered();
        sleep(1200);
        robot.setClawClosed();
        sleep(400);
        robot.setRetractedUp();
        robot.moveBase(0.6);
        sleep(1500);
        robot.moveTop(0.7);
        robot.moveWrist(0.25);
        sleep(500);
        robot.moveBase(0.14);
        robot.setIntakeUp();
        sleep(2500);

        if (randomization == 2) {
            Actions.runBlocking(act13);
        } else if (randomization == 1) {
            Actions.runBlocking(act23);
        } else {
            Actions.runBlocking(act33);
        }

        robot.setClawOpen();
        sleep(800);

        if (randomization == 2) {
            Actions.runBlocking(act14);
        } else if (randomization == 1) {
            Actions.runBlocking(act24);
        } else {
            Actions.runBlocking(act34);
        }

        robot.setRetracted();
        sleep(500);

        if (randomization == 2) {
            Actions.runBlocking(act15);
        } else if (randomization == 1) {
            Actions.runBlocking(act25);
        } else {
            Actions.runBlocking(act35);
        }
    }
}