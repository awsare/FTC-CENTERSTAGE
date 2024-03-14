package org.firstinspires.ftc.teamcode.opmodes.statesauto;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.ActionRobot;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.vision.PropCamera;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Blue WING \uD83D\uDC0B", group = "blue")
public class BlueWing extends LinearOpMode {

    MecanumDrive drive;
    ActionRobot robot;
    PropCamera camera;

    int randomization = 0;
    String park = "wall";

    Pose2d startPose = new Pose2d(-37, 61, Math.toRadians(-90));

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);

        robot = new ActionRobot();
        robot.init(hardwareMap, false);

        robot.setIntakeDown();
        robot.setClawClosed();
        sleep(2000);
        robot.setRetracted();
        robot.moveBase(0.5);

        camera = new PropCamera(hardwareMap, telemetry, "Blue", "Right");

        Action act01 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-36, 37), Math.toRadians(-10))
                .build();

        Action act02 = drive.actionBuilder(new Pose2d(-36, 37, Math.toRadians(-10)))
                .strafeToLinearHeading(new Vector2d(-47,50), 0)
                .strafeTo(new Vector2d(-37, 61.5))
                .strafeTo(new Vector2d(25, 61.5))
                .strafeTo(new Vector2d(42, 46.5))
                .build();

        Action act03 = drive.actionBuilder(new Pose2d(42, 46.5, Math.toRadians(0)))
                .strafeTo(new Vector2d(47.5, 46.5))
                .build();

        Action act04 = drive.actionBuilder(new Pose2d(47.5, 46.5, Math.toRadians(0)))
                .strafeTo(new Vector2d(42, 46.5))
                .build();

        Action act051 = drive.actionBuilder(new Pose2d(42, 46.5, Math.toRadians(0)))
                .strafeTo(new Vector2d(42, 64))
                .strafeTo(new Vector2d(55, 64))
                .build();

        Action act052 = drive.actionBuilder(new Pose2d(42, 46.5, Math.toRadians(0)))
                .strafeTo(new Vector2d(42, 17))
                .strafeTo(new Vector2d(58, 17))
                .build();

        Action act11 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-39, 38))
                .build();

        Action act12 = drive.actionBuilder(new Pose2d(-39, 38, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-47,50))
                .turnTo(0)
                .strafeTo(new Vector2d(-37, 61.5))
                .strafeTo(new Vector2d(25, 61.5))
                .strafeTo(new Vector2d(42, 40))
                .build();

        Action act13 = drive.actionBuilder(new Pose2d(42, 40, Math.toRadians(0)))
                .strafeTo(new Vector2d(47.5, 40))
                .build();

        Action act14 = drive.actionBuilder(new Pose2d(47.5, 40, Math.toRadians(0)))
                .strafeTo(new Vector2d(42, 40))
                .build();

        Action act151 = drive.actionBuilder(new Pose2d(42, 40, Math.toRadians(0)))
                .strafeTo(new Vector2d(42, 64))
                .strafeTo(new Vector2d(55, 64))
                .build();

        Action act152 = drive.actionBuilder(new Pose2d(42, 40, Math.toRadians(0)))
                .strafeTo(new Vector2d(42, 17))
                .strafeTo(new Vector2d(58, 17))
                .build();

        Action act21 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-46, 48.5))
                .build();

        Action act22 = drive.actionBuilder(new Pose2d(-46, 48.5, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-47, 51))
                .turnTo(0)
                .strafeTo(new Vector2d(-37, 61.5))
                .strafeTo(new Vector2d(25, 61.5))
                .strafeTo(new Vector2d(42, 33))
                .build();

        Action act23 = drive.actionBuilder(new Pose2d(42, 33, Math.toRadians(0)))
                .strafeTo(new Vector2d(47.5, 33))
                .build();

        Action act24 = drive.actionBuilder(new Pose2d(47.5, 33, Math.toRadians(0)))
                .strafeTo(new Vector2d(42, 33))
                .build();

        Action act251 = drive.actionBuilder(new Pose2d(42, 33, Math.toRadians(0)))
                .strafeTo(new Vector2d(42, 64))
                .strafeTo(new Vector2d(55, 64))
                .build();

        Action act252 = drive.actionBuilder(new Pose2d(42, 33, Math.toRadians(0)))
                .strafeTo(new Vector2d(42, 17))
                .strafeTo(new Vector2d(58, 17))
                .build();

        while (opModeInInit()) {
            if (gamepad1.cross) {
                park = "center";
            }
            if (gamepad1.circle) {
                park = "wall";
            }

            randomization = camera.getRandomization();
            telemetry.addData("Randomization", randomization);
            telemetry.addData("Park Side", park);
            telemetry.update();
        }

        waitForStart();

        robot.moveBase(0);
        robot.moveTop(0.5);
        robot.moveWrist(0.485);

        if (randomization == 0) {
            Actions.runBlocking(act01);
        } else if (randomization == 1) {
            Actions.runBlocking(act11);
        } else {
            Actions.runBlocking(act21);
        }

        robot.setClawScoreOpen();
        sleep(400);
        robot.setClawClosed();
        sleep(200);
        robot.setRetracted();
        sleep(800);

        Actions.runBlocking(robot.transfer());

        if (randomization == 0) {
            Actions.runBlocking(act02);
        } else if (randomization == 1) {
            Actions.runBlocking(act12);
        } else {
            Actions.runBlocking(act22);
        }

        Actions.runBlocking(robot.deposit());

        if (randomization == 0) {
            Actions.runBlocking(act03);
        } else if (randomization == 1) {
            Actions.runBlocking(act13);
        } else {
            Actions.runBlocking(act23);
        }

        robot.setClawOpen();
        sleep(500);

        if (randomization == 0) {
            Actions.runBlocking(act04);
        } else if (randomization == 1) {
            Actions.runBlocking(act14);
        } else {
            Actions.runBlocking(act24);
        }

        robot.setRetracted();

        if (park.equals("wall")) {
            if (randomization == 0) {
                Actions.runBlocking(act051);
            } else if (randomization == 1) {
                Actions.runBlocking(act151);
            } else {
                Actions.runBlocking(act251);
            }
        } else {
            if (randomization == 0) {
                Actions.runBlocking(act052);
            } else if (randomization == 1) {
                Actions.runBlocking(act152);
            } else {
                Actions.runBlocking(act252);
            }
        }

        camera.stopStreaming();
    }
}