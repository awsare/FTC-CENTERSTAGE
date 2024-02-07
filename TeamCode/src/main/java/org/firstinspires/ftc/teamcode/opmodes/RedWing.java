package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.vision.PropCamera;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Red Wing \uD83E\uDD91 Right Park", group = "red")
public class RedWing extends LinearOpMode {

    MecanumDrive drive;
    Robot robot;
    PropCamera camera;

    int randomization = 0;

    Pose2d startPose = new Pose2d(-37, -61, Math.toRadians(90));

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);

        robot = new Robot();
        robot.init(hardwareMap, false);

        camera = new PropCamera(hardwareMap, telemetry, "Red", "Right");

        Action act11 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-47, -47))
                .build();

        Action act12 = drive.actionBuilder(new Pose2d(-47, -47, Math.toRadians(90)))
                .strafeTo(new Vector2d(-47,-50))
                .turnTo(0)
                .strafeTo(new Vector2d(-37, -60))
                .strafeTo(new Vector2d(25, -60))
                .strafeTo(new Vector2d(42, -29.5))
                .build();

        Action act13 = drive.actionBuilder(new Pose2d(42, -29.5, Math.toRadians(0)))
                .strafeTo(new Vector2d(47.5, -29.5))
                .build();

        Action act14 = drive.actionBuilder(new Pose2d(47.5, -29.5, Math.toRadians(0)))
                .strafeTo(new Vector2d(42, -29.5))
                .build();

        Action act15 = drive.actionBuilder(new Pose2d(42, -29.5, Math.toRadians(0)))
                .strafeTo(new Vector2d(45, -61))
                .build();

        Action act21 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-39, -37))
                .build();

        Action act22 = drive.actionBuilder(new Pose2d(-39, -37, Math.toRadians(90)))
                .strafeTo(new Vector2d(-47,-50))
                .turnTo(0)
                .strafeTo(new Vector2d(-37, -60))
                .strafeTo(new Vector2d(25, -60))
                .strafeTo(new Vector2d(42, -36))
                .build();

        Action act23 = drive.actionBuilder(new Pose2d(42, -36, Math.toRadians(0)))
                .strafeTo(new Vector2d(47.5, -36))
                .build();

        Action act24 = drive.actionBuilder(new Pose2d(47.5, -36, Math.toRadians(0)))
                .strafeTo(new Vector2d(42, -36))
                .build();

        Action act25 = drive.actionBuilder(new Pose2d(42, -36, Math.toRadians(0)))
                .strafeTo(new Vector2d(45, -61))
                .build();

        Action act31 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-34, -38), Math.toRadians(30))
                .build();

        Action act32 = drive.actionBuilder(new Pose2d(-34, -38, Math.toRadians(-30)))
                .strafeToLinearHeading(new Vector2d(-47,-50), 0)
                .strafeTo(new Vector2d(-37, -60))
                .strafeTo(new Vector2d(25, -60))
                .strafeTo(new Vector2d(42, -42.5))
                .build();

        Action act33 = drive.actionBuilder(new Pose2d(42, -42.5, Math.toRadians(0)))
                .strafeTo(new Vector2d(47.5, -42.5))
                .build();

        Action act34 = drive.actionBuilder(new Pose2d(47.5, -42.5, Math.toRadians(0)))
                .strafeTo(new Vector2d(42, -42.5))
                .build();

        Action act35 = drive.actionBuilder(new Pose2d(42, -36, Math.toRadians(0)))
                .strafeTo(new Vector2d(45, -61))
                .build();

        robot.setClawClosed();
        sleep(3000);
        robot.setRetracted();
        robot.moveBase(0.5);

        while (opModeInInit()) {
            randomization = camera.getRandomization();
            telemetry.addData("Randomization", randomization);
            telemetry.update();
        }

        waitForStart();

        camera.stopStreaming();

        // *** REMOVE ***
        randomization = 2;
        // *** REMOVE ***

        telemetry.addData("Randomization", randomization);
        telemetry.update();

        if (randomization == 0) {
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
        sleep(700);
        robot.setClawScoreOpen();
        sleep(400);
        robot.setClawClosed();
        sleep(200);
        robot.setRetractedUp();
        sleep(600);
        robot.setRetracted();
        sleep(500);
        robot.setClawOpen();
        sleep(250);
        robot.setRetractedLowered();
        sleep(500);
        robot.setClawClosed();
        sleep(400);
        robot.setRetractedUp();
        sleep(300);
        robot.setIntakeUp();
        sleep(300);

        if (randomization == 0) {
            Actions.runBlocking(act12);
        } else if (randomization == 1) {
            Actions.runBlocking(act22);
        } else {
            Actions.runBlocking(act32);
        }

        robot.setIntakeDown();
        sleep(600);
        robot.moveBase(0.5);
        sleep(800);
        robot.setIntakeUp();
        robot.moveTop(0.7);
        robot.moveWrist(0.25);
        sleep(500);
        robot.moveBase(0.14);
        sleep(1000);

        if (randomization == 0) {
            Actions.runBlocking(act13);
        } else if (randomization == 1) {
            Actions.runBlocking(act23);
        } else {
            Actions.runBlocking(act33);
        }

        robot.setClawScoreOpen();
        sleep(1000);

        if (randomization == 0) {
            Actions.runBlocking(act14);
        } else if (randomization == 1) {
            Actions.runBlocking(act24);
        } else {
            Actions.runBlocking(act34);
        }

        robot.setRetracted();

        if (randomization == 0) {
            Actions.runBlocking(act15);
        } else if (randomization == 1) {
            Actions.runBlocking(act25);
        } else {
            Actions.runBlocking(act35);
        }
    }
}