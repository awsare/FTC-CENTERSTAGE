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
public class FullRedWing extends LinearOpMode {

    MecanumDrive drive;
    Robot robot;
    PropCamera camera;

    int randomization = 0;

    Pose2d startPose = new Pose2d(-37, -62, Math.PI / 2.0);

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);
        robot = new Robot();
        robot.init(hardwareMap, false);

        camera = new PropCamera(hardwareMap, telemetry, "Red", "Left");

        Action toScoreA = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(-37, -17))
                .turnTo(Math.toRadians(-120))
                .build();

        Action backA = drive.actionBuilder(new Pose2d(-37, -17, Math.toRadians(-120)))
                .turnTo(Math.toRadians(20))
                .strafeToConstantHeading(new Vector2d(-37, -5))
                .strafeToConstantHeading(new Vector2d(50, -5))
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

        if (randomization == 0) {
            Actions.runBlocking(toScoreA);
        } else if (randomization == 2) {
            //Actions.runBlocking(toTwo);
        } else {
            //Actions.runBlocking(toThree);
        }

        robot.setScoring();
        robot.moveTop(0.7);
        sleep(1500);
        robot.setClawScoreOpen();
        sleep(1000);
        robot.setClawClosed();
        robot.setRetracted();
        sleep(1000);

        if (randomization == 0) {
            Actions.runBlocking(backA);
        } else if (randomization == 2) {
            //Actions.runBlocking(backTwo);
        } else {
            //Actions.runBlocking(backThree);
        }
    }
}
