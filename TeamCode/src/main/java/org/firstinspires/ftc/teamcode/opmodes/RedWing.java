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

@Autonomous(name = "Red Wing \uD83E\uDD91 ", group = "red")
public class RedWing extends LinearOpMode {

    MecanumDrive drive;
    Robot robot;
    PropCamera camera;

    int randomization = 2;

    Pose2d startPose = new Pose2d(-37, -61, Math.toRadians(90));

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);

        robot = new Robot();
        robot.init(hardwareMap, false);

        camera = new PropCamera(hardwareMap, telemetry, "Red", "Left");

        Action act11 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-47, -47))
                .build();

        Action act12 = drive.actionBuilder(new Pose2d(-47, -47, Math.toRadians(90)))
                .strafeTo(new Vector2d(-47,-50))
                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(-37, -61))
                .strafeTo(new Vector2d(25, -61))
                .strafeTo(new Vector2d(42, -28))
//                .strafeTo(new Vector2d(45, -28))
//                .strafeTo(new Vector2d(44, -28))
                //.strafeTo(new Vector2d(48, -11))
                .build();

        Action act21 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(17, 38))
                .build();

        Action act31 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(23.5, 48))
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

        telemetry.addData("Randomization", randomization);
        telemetry.update();

        if (randomization == 2) {
            Actions.runBlocking(act11);
        } else if (randomization == 1) {
            Actions.runBlocking(act21);
        } else {
            Actions.runBlocking(act31);
        }

        robot.moveBase(0.1);
        robot.moveTop(0.45);
        robot.moveWrist(0.275);
        sleep(700);
        robot.setClawScoreOpen();
        sleep(400);
        robot.setClawClosed();
        sleep(200);
        robot.setRetracted();
        sleep(800);
        robot.setClawOpen();
        sleep(250);
        robot.setRetractedLowered();
        sleep(500);
        robot.setClawClosed();
        sleep(400);
        robot.setRetractedUp();
        robot.setIntakeUp();
        sleep(300);

        if (randomization == 2) {
            Actions.runBlocking(act12);
        } else if (randomization == 1) {
            Actions.runBlocking(act21);
        } else {
            Actions.runBlocking(act31);
        }

        robot.setIntakeDown();
        sleep(300);
        robot.moveBase(0.6);
        sleep(1500);
        robot.moveTop(0.7);
        robot.moveWrist(0.25);
        sleep(500);
        robot.moveBase(0.14);
        sleep(2500);
    }
}