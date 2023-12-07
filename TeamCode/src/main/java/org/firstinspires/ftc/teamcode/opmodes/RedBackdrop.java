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
public class RedBackdrop extends LinearOpMode {

    MecanumDrive drive;
    Robot robot;
    //PropCamera camera;

    double randomization = 2;

    Pose2d startPose = new Pose2d(12, -62, Math.toRadians(270));

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);

        robot = new Robot();
        robot.init(hardwareMap, false);

        //camera = new PropCamera(hardwareMap, telemetry, "Red");

        Action scorePurpleZero = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(20, -39))
                .turnTo(Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(3, -39))
                .strafeToConstantHeading(new Vector2d(47, -39))
                .build();

        Action scorePurpleOne = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(18, -26))
                .strafeToConstantHeading(new Vector2d(18, -41))
                .strafeToConstantHeading(new Vector2d(47, -41))
                .build();

        Action scorePurpleTwo = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(29.5, -34))
                .strafeToConstantHeading(new Vector2d(29.5, -49))
                .strafeToConstantHeading(new Vector2d(55, -49))
                .build();

//        while (opModeInInit()) {
//            randomization = camera.getRandomization();
//            telemetry.addData("Randomization", randomization);
//            telemetry.update();
//        }

        waitForStart();

//        camera.stopStreaming();

        telemetry.addData("Randomization", randomization);
        telemetry.update();

        robot.setIntakeAngle(StandardTeleOp.INTAKE_UP);
        sleep(1000);
//        robot.setIntakeAngle(StandardTeleOp.INTAKE_DOWN);
//        sleep(1000);
//
//        robot.powerIntake(-0.3);
//        sleep(300);
//        robot.powerIntake(0);

        if (randomization == 0) {
            Actions.runBlocking(scorePurpleZero);
        } else if (randomization == 1) {
            Actions.runBlocking(scorePurpleOne);
        } else {
            Actions.runBlocking(scorePurpleTwo);
        }

        robot.setIntakeAngle(StandardTeleOp.INTAKE_UP);

        sleep(1000);
    }
}
