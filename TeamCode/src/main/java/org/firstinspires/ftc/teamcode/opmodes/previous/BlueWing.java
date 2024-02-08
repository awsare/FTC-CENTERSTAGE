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
@Autonomous(name = "New Blue Wing \uD83D\uDC0B", group = "blue")
public class BlueWing extends LinearOpMode {

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

        camera = new PropCamera(hardwareMap, telemetry, "Blue", "Right");

        Action act11 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(10.5, -41), Math.toRadians(150))
                .build();

        Action act21 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(15, -38))
                .build();

        Action act31 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(22, -48))
                .build();

        robot.setClawClosed();
        sleep(1000);
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

        if (randomization == 0) {
            Actions.runBlocking(act11);
        } else if (randomization == 1) {
            Actions.runBlocking(act21);
        } else {
            Actions.runBlocking(act31);
        }

        robot.moveBase(0.1);
        robot.moveTop(0.45);
        robot.moveWrist(0.275);
        sleep(1500);
        robot.setClawScoreOpen();
        sleep(1000);
        robot.setClawClosed();
        robot.setRetracted();
        sleep(1000);
    }
}