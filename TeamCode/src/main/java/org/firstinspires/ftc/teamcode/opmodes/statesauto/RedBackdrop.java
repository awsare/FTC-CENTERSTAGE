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

@Autonomous(name = "Red BACKDROP \uD83E\uDD91", group = "red")
public class RedBackdrop extends LinearOpMode {

    MecanumDrive drive;
    ActionRobot robot;
    PropCamera camera;

    Gamepad driver;
    Gamepad operator;

    int randomization = 0;
    String park = "wall";

    Pose2d startPose = new Pose2d(13, -62, Math.PI / 2.0);

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);

        robot = new ActionRobot();
        robot.init(hardwareMap, false);

        driver = new Gamepad();
        operator = new Gamepad();

        setDefault();

        robot.setClawClosed();
        robot.setIntakeDown();
        sleep(2000);
        robot.setRetracted();
        robot.moveBase(0.5);

        camera = new PropCamera(hardwareMap, telemetry, "Red", "Right");

        Action act01 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(10, -45), Math.toRadians(137))
                .build();

        Action act02 = drive.actionBuilder(new Pose2d(10, -45, Math.toRadians(137)))
                .afterTime(0.45, robot.transfer())
                .afterTime(1.5, robot.deposit())
                .strafeToLinearHeading(new Vector2d(40, -30), 0)
                .build();

        Action act03 = drive.actionBuilder(new Pose2d(40, -30, 0))
                .strafeTo(new Vector2d(45, -30))
                .build();

        Action act04 = drive.actionBuilder(new Pose2d(45, -30, 0))
                .strafeTo(new Vector2d(42, -30))
                .build();

        Action act051 = drive.actionBuilder(new Pose2d(42, -30, 0))
                .strafeTo(new Vector2d(47, -11))
                .strafeTo(new Vector2d(58, -11))
                .build();

        Action act052 = drive.actionBuilder(new Pose2d(42, -30, 0))
                .strafeTo(new Vector2d(47, -61.5))
                .strafeTo(new Vector2d(58, -61.5))
                .build();

        Action act11 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(15, -40))
                .build();

        Action act12 = drive.actionBuilder(new Pose2d(15, -40, Math.toRadians(90)))
                .afterTime(0.45, robot.transfer())
                .afterTime(1.5, robot.deposit())
                .strafeToLinearHeading(new Vector2d(40, -37.5), 0)
                .build();

        Action act13 = drive.actionBuilder(new Pose2d(40, -37.5, 0))
                .strafeTo(new Vector2d(45, -37.5))
                .build();

        Action act14 = drive.actionBuilder(new Pose2d(45, -37.5, 0))
                .strafeTo(new Vector2d(42, -37.5))
                .build();

        Action act151 = drive.actionBuilder(new Pose2d(42, -37.5, 0))
                .strafeTo(new Vector2d(47, -11))
                .strafeTo(new Vector2d(58, -11))
                .build();

        Action act152 = drive.actionBuilder(new Pose2d(42, -37.5, 0))
                .strafeTo(new Vector2d(47, -61.5))
                .strafeTo(new Vector2d(58, -61.5))
                .build();

        Action act21 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(23, -48))
                .build();

        Action act22 = drive.actionBuilder(new Pose2d(23, -48, Math.toRadians(90)))
                .afterTime(0.45, robot.transfer())
                .afterTime(1.5, robot.deposit())
                .strafeTo(new Vector2d(30, -49))
                .strafeToLinearHeading(new Vector2d(40, -43), 0)
                .build();

        Action act23 = drive.actionBuilder(new Pose2d(40, -43, 0))
                .strafeTo(new Vector2d(45, -43))
                .build();

        Action act24 = drive.actionBuilder(new Pose2d(45, -43, 0))
                .strafeTo(new Vector2d(42, -43))
                .build();

        Action act251 = drive.actionBuilder(new Pose2d(42, -43, 0))
                .strafeTo(new Vector2d(47, -11))
                .strafeTo(new Vector2d(58, -11))
                .build();

        Action act252 = drive.actionBuilder(new Pose2d(42, -43, 0))
                .strafeTo(new Vector2d(47, -61.5))
                .strafeTo(new Vector2d(58, -61.5))
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

        setPurple();

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

        setDefault();

        if (randomization == 0) {
            Actions.runBlocking(act02);
        } else if (randomization == 1) {
            Actions.runBlocking(act12);
        } else {
            Actions.runBlocking(act22);
        }

        setYellow();

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

        setDefault();
        robot.setRetracted();

        if (park.equals("center")) {
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

    void setDefault() {
        gamepad1.setLedColor(255, 255, 255, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(255, 255, 255, Gamepad.LED_DURATION_CONTINUOUS);
    }

    void setPurple() {
        gamepad1.setLedColor(120, 0, 120, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(120, 0, 120, Gamepad.LED_DURATION_CONTINUOUS);
    }
    void setYellow() {
        gamepad1.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
    }
}
