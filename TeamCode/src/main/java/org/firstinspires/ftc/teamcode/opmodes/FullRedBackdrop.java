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

    double randomization;

    Pose2d startPose = new Pose2d(14, -62, Math.PI / 2.0);

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);

        robot = new Robot();
        robot.init(hardwareMap, false);

        //camera = new PropCamera(hardwareMap, telemetry, "Red");

        Action act1 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(14, -58))
                .strafeTo(new Vector2d(35, -45))
                .turnTo(Math.toRadians(-2))
                .build();

        Action goToYellowA = drive.actionBuilder(new Pose2d(35, -45, Math.toRadians(-2)))
                .strafeTo(new Vector2d(35, -27))
                .build();

        Action scoreYellowA = drive.actionBuilder(new Pose2d(35, -27, Math.toRadians(-2)))
                .strafeTo(new Vector2d(44.25, -27))
                .build();

        Action goToYellowB = drive.actionBuilder(new Pose2d(35, -45, Math.toRadians(-2)))
                .strafeTo(new Vector2d(35, -35))
                .build();

        Action scoreYellowB = drive.actionBuilder(new Pose2d(35, -35, Math.toRadians(-2)))
                .strafeTo(new Vector2d(44.25, -35))
                .build();

        Action goToYellowC = drive.actionBuilder(new Pose2d(35, -45, Math.toRadians(-2)))
                .strafeTo(new Vector2d(35, -41.5))
                .build();

        Action scoreYellowC = drive.actionBuilder(new Pose2d(35, -41.5, Math.toRadians(-2)))
                .strafeTo(new Vector2d(44.25, -41.5))
                .build();

//        while (opModeInInit()) {
//            randomization = camera.getRandomization();
//            telemetry.addData("Randomization", randomization);
//            telemetry.update();
//        }

        robot.setClawClosed();
        sleep(2000);
        robot.moveBase(0.7);

        waitForStart();

        //camera.stopStreaming();

        telemetry.addData("Randomization", randomization);
        telemetry.update();

        Actions.runBlocking(act1);
        robot.setScoring();
        Actions.runBlocking(goToYellowA);
        Actions.runBlocking(scoreYellowA);
        robot.setScoringLifted();
        robot.setClawScoreOpen();
        sleep(900);
        robot.setRetracted();
        sleep(500);
    }
}
