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
public class FullRedBackdrop extends LinearOpMode {

    MecanumDrive drive;
    Robot robot;
    PropCamera camera;

    int randomization = 0;

    Pose2d startPose = new Pose2d(14, -62, Math.PI / 2.0);

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);

        robot = new Robot();
        robot.init(hardwareMap, false);

        camera = new PropCamera(hardwareMap, telemetry, "Red");

        Action act1 = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(14, -58))
                .strafeToConstantHeading(new Vector2d(35, -45))
                .turnTo(Math.toRadians(-5))
                .build();

        Action goToYellowA = drive.actionBuilder(new Pose2d(35, -45, 0))
                .strafeToConstantHeading(new Vector2d(35, -29))
                .strafeToConstantHeading(new Vector2d(44.25, -29))
                .build();

        Action backYellowA = drive.actionBuilder(new Pose2d(44.25, -29, 0))
                .strafeToConstantHeading(new Vector2d(40, -29))
                .build();

        Action goToYellowB = drive.actionBuilder(new Pose2d(35, -45, 0))
                .strafeToConstantHeading(new Vector2d(35, -35))
                .strafeToConstantHeading(new Vector2d(44.25, -35))
                .build();

        Action backYellowB = drive.actionBuilder(new Pose2d(44.25, -35, 0))
                .strafeToConstantHeading(new Vector2d(40, -35))
                .build();

        Action goToYellowC = drive.actionBuilder(new Pose2d(35, -45, 0))
                .strafeToConstantHeading(new Vector2d(35, -41.5))
                .strafeToConstantHeading(new Vector2d(44.25, -41.5))
                .build();

        Action backYellowC = drive.actionBuilder(new Pose2d(44.25, -41.5, 0))
                .strafeToConstantHeading(new Vector2d(40, -41.5))
                .build();

        Action purpleA = drive.actionBuilder(new Pose2d(44.25, -29, 0))
                .strafeToConstantHeading(new Vector2d(18, -32.5))
                .strafeToConstantHeading(new Vector2d(0.5, -32.5))
                .strafeToConstantHeading(new Vector2d(18, -32.5))
                .strafeToConstantHeading(new Vector2d(49, -14))
                .build();

        Action purpleB = drive.actionBuilder(new Pose2d(40, -35, 0))
                .strafeToConstantHeading(new Vector2d(20, -22))
                .strafeToConstantHeading(new Vector2d(12, -22))
                .strafeToConstantHeading(new Vector2d(20, -22))
                .strafeToConstantHeading(new Vector2d(49, -14))
                .build();

        Action purpleC = drive.actionBuilder(new Pose2d(40, -41.5, 0))
                .strafeToConstantHeading(new Vector2d(30, -27))
                .strafeToConstantHeading(new Vector2d(20, -27))
                .strafeToConstantHeading(new Vector2d(30, -27))
                .strafeToConstantHeading(new Vector2d(49, -14))
                .build();

        robot.setClawClosed();
        sleep(1000);
        robot.moveBase(0.7);
        robot.setCageDown();

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
        robot.setCageUp();

        if (randomization == 0) {
            Actions.runBlocking(purpleA);
        } else if (randomization == 1) {
            Actions.runBlocking(purpleB);
        } else {
            Actions.runBlocking(purpleC);
        }

    }
}