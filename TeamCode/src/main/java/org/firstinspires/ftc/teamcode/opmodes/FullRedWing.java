package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
public class FullRedWing extends LinearOpMode {

    MecanumDrive drive;
    Robot robot;

    Pose2d startPose = new Pose2d(-37, -62, Math.PI / 2.0);

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);
        robot = new Robot();
        robot.init(hardwareMap, false);

        Action toScore = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(-36, -37))
                .build();

        Action park = drive.actionBuilder(new Pose2d(-36, -37, Math.PI / 2.0))
                .strafeToConstantHeading(new Vector2d(-36, -12))
                .turnTo(0)
                .strafeToConstantHeading(new Vector2d(50, -12))
                .build();

        robot.setClawClosed();
        robot.moveBase(0.75);

        waitForStart();

        Actions.runBlocking(toScore);

        robot.setScoring();
        robot.moveTop(0.7);
        sleep(1500);
        robot.setClawScoreOpen();
        sleep(1000);
        robot.setRetracted();
        sleep(1000);
    }
}
