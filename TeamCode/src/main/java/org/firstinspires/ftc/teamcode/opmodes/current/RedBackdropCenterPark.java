package org.firstinspires.ftc.teamcode.opmodes.current;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.vision.PropCamera;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Red BACKDROP \uD83E\uDD91 Center Park", group = "red")
public class RedBackdropCenterPark extends LinearOpMode {

    MecanumDrive drive;
    Robot robot;
    PropCamera camera;

    int randomization = 0;

    Pose2d startPose = new Pose2d(13, -62, Math.PI / 2.0);

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);

        robot = new Robot();
        robot.init(hardwareMap, false);

        robot.setClawClosed();
        sleep(3000);
        robot.setRetracted();
        robot.moveBase(0.5);

        camera = new PropCamera(hardwareMap, telemetry, "Red", "Right");

        Action act01 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(12, -38), Math.toRadians(170))
                .build();

        Action act02 = drive.actionBuilder(new Pose2d(12, -38, Math.toRadians(170)))
                .strafeToLinearHeading(new Vector2d(40, -30), 0)
                .build();

        Action act03 = drive.actionBuilder(new Pose2d(40, -30, 0))
                .strafeTo(new Vector2d(45, -30))
                .build();

        Action act04 = drive.actionBuilder(new Pose2d(45, -30, 0))
                .strafeTo(new Vector2d(42, -30))
                .build();

        Action act05 = drive.actionBuilder(new Pose2d(42, -30, 0))
                .strafeTo(new Vector2d(47, -9))
                .build();

        Action act11 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(15, -39))
                .build();

        Action act12 = drive.actionBuilder(new Pose2d(15, -39, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(40, -37), 0)
                .build();

        Action act13 = drive.actionBuilder(new Pose2d(40, -37, 0))
                .strafeTo(new Vector2d(45, -37))
                .build();

        Action act14 = drive.actionBuilder(new Pose2d(45, -37, 0))
                .strafeTo(new Vector2d(42, -37))
                .build();

        Action act15 = drive.actionBuilder(new Pose2d(42, -37, 0))
                .strafeTo(new Vector2d(47, -9))
                .build();

        Action act21 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(23, -49))
                .build();

        Action act22 = drive.actionBuilder(new Pose2d(23, -49, Math.toRadians(90)))
                .strafeTo(new Vector2d(30, -49))
                .strafeToLinearHeading(new Vector2d(40, -43), 0)
                .build();

        Action act23 = drive.actionBuilder(new Pose2d(40, -43, 0))
                .strafeTo(new Vector2d(45, -43))
                .build();

        Action act24 = drive.actionBuilder(new Pose2d(45, -43, 0))
                .strafeTo(new Vector2d(42, -43))
                .build();

        Action act25 = drive.actionBuilder(new Pose2d(42, -43, 0))
                .strafeTo(new Vector2d(47, -9))
                .build();

        while (opModeInInit()) {
            randomization = camera.getRandomization();
            telemetry.addData("Randomization", randomization);
            telemetry.update();
        }

        waitForStart();

        robot.setIntakeDown();
        robot.moveBase(0);
        robot.moveTop(0.5);
        robot.moveWrist(0.275);

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
            Actions.runBlocking(act02);
        } else if (randomization == 1) {
            Actions.runBlocking(act12);
        } else {
            Actions.runBlocking(act22);
        }

        robot.setIntakeDown();
        sleep(600);
        robot.moveBase(0.4);
        sleep(800);
        robot.setIntakeUp();
        robot.moveTop(0.6);
        robot.moveWrist(0.35);
        sleep(500);
        robot.moveBase(0.14);
        sleep(1000);

        if (randomization == 0) {
            Actions.runBlocking(act03);
        } else if (randomization == 1) {
            Actions.runBlocking(act13);
        } else {
            Actions.runBlocking(act23);
        }

        robot.setClawAutoOpen();
        sleep(500);

        if (randomization == 0) {
            Actions.runBlocking(act04);
        } else if (randomization == 1) {
            Actions.runBlocking(act14);
        } else {
            Actions.runBlocking(act24);
        }

        robot.setRetracted();

        if (randomization == 0) {
            Actions.runBlocking(act05);
        } else if (randomization == 1) {
            Actions.runBlocking(act15);
        } else {
            Actions.runBlocking(act25);
        }

        camera.stopStreaming();
    }
}
