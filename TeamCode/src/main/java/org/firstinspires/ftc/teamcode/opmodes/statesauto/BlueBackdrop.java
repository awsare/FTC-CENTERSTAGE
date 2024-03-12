package org.firstinspires.ftc.teamcode.opmodes.statesauto;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.ActionRobot;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.vision.PropCamera;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Blue BACKDROP \uD83D\uDC0B", group = "blue")
public class BlueBackdrop extends LinearOpMode {

    MecanumDrive drive;
    ActionRobot robot;
    PropCamera camera;

    Gamepad driver;
    Gamepad operator;

    int randomization = 0;
    String park = "wall";

    Pose2d startPose = new Pose2d(13, 62, -Math.PI / 2.0);

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);

        robot = new ActionRobot();
        robot.init(hardwareMap, false);

        driver = new Gamepad();
        operator = new Gamepad();

        gamepad1.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(255, 255, 255, Gamepad.LED_DURATION_CONTINUOUS);

        robot.setClawClosed();
        sleep(3000);
        robot.setRetracted();
        robot.moveBase(0.5);

        camera = new PropCamera(hardwareMap, telemetry, "Blue", "Left");

        Action act01 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(23, 48))
                .build();

        Action act02 = drive.actionBuilder(new Pose2d(23, 48, Math.toRadians(-90)))
                .afterTime(0.45, robot.transfer())
                .afterTime(1.5, robot.deposit())
                .strafeTo(new Vector2d(30, 50))
                .strafeToLinearHeading(new Vector2d(37, 47), 0)
                .build();

        Action act03 = drive.actionBuilder(new Pose2d(37, 47, 0))
                .strafeTo(new Vector2d(44, 47))
                .build();

        Action act04 = drive.actionBuilder(new Pose2d(44, 47, 0))
                .strafeTo(new Vector2d(37, 47))
                .build();

        Action act051 = drive.actionBuilder(new Pose2d(37, 47, 0))
                .strafeTo(new Vector2d(42, 65))
                .strafeTo(new Vector2d(55, 65))
                .build();

        Action act052 = drive.actionBuilder(new Pose2d(37, 47, 0))
                .strafeTo(new Vector2d(42, 14))
                .strafeTo(new Vector2d(55, 14))
                .build();

        Action act11 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(17, 39))
                .build();

        Action act12 = drive.actionBuilder(new Pose2d(17, 39, Math.toRadians(-90)))
                .afterTime(0.45, robot.transfer())
                .afterTime(1.5, robot.deposit())
                .strafeToLinearHeading(new Vector2d(41, 40), 0)
                .build();

        Action act13 = drive.actionBuilder(new Pose2d(41, 40, 0))
                .strafeTo(new Vector2d(44, 40))
                .build();

        Action act14 = drive.actionBuilder(new Pose2d(44, 40, 0))
                .strafeTo(new Vector2d(41, 40))
                .build();

        Action act151 = drive.actionBuilder(new Pose2d(41, 40, 0))
                .strafeTo(new Vector2d(42, 65))
                .strafeTo(new Vector2d(55, 65))
                .build();

        Action act152 = drive.actionBuilder(new Pose2d(41, 40, 0))
                .strafeTo(new Vector2d(42, 14))
                .strafeTo(new Vector2d(55, 14))
                .build();

        Action act21 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(10, 45), Math.toRadians(-137))
                .build();

        Action act22 = drive.actionBuilder(new Pose2d(10, 45, Math.toRadians(-137)))
                .afterTime(0.45, robot.transfer())
                .afterTime(1.5, robot.deposit())
                .strafeToLinearHeading(new Vector2d(37, 32.5), 0)
                .build();

        Action act23 = drive.actionBuilder(new Pose2d(37, 32.5, 0))
                .strafeTo(new Vector2d(44.5, 32.5))
                .build();

        Action act24 = drive.actionBuilder(new Pose2d(44.5, 32.5, 0))
                .strafeTo(new Vector2d(37, 32.5))
                .build();

        Action act251 = drive.actionBuilder(new Pose2d(37, 32.5, 0))
                .strafeTo(new Vector2d(42, 65))
                .strafeTo(new Vector2d(55, 65))
                .build();

        Action act252 = drive.actionBuilder(new Pose2d(37, 32.5, 0))
                .strafeTo(new Vector2d(42, 14))
                .strafeTo(new Vector2d(55, 14))
                .build();

        while (opModeInInit()) {
            if (driver.cross) {
                park = "center";
            }
            if (driver.circle) {
                park = "wall";
            }

            randomization = camera.getRandomization();
            telemetry.addData("Randomization", randomization);
            telemetry.addData("Park Side", park);
            telemetry.addData("Input", driver.left_stick_x);
            telemetry.update();
        }

        waitForStart();

        robot.setIntakeDown();
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
        sleep(300);
        robot.setClawClosed();
        robot.setRetracted();

        if (randomization == 0) {
            Actions.runBlocking(act02);
        } else if (randomization == 1) {
            Actions.runBlocking(act12);
        } else {
            Actions.runBlocking(act22);
        }

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
