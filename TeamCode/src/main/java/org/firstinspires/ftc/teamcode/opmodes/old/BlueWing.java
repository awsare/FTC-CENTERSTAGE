package org.firstinspires.ftc.teamcode.opmodes.old;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.opmodes.StandardTeleOp;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
@Disabled
@Autonomous(name = "Blue Wing (Park) \uD83D\uDC0B", group = "blue")
public class BlueWing extends LinearOpMode {

    MecanumDrive drive;
    Robot robot;

    Pose2d startPose = new Pose2d(-37, 62, Math.toRadians(90));

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);
        robot = new Robot();
        robot.init(hardwareMap, false);

        Action toBackstage = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(-37, 17))
                .turnTo(Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(50, 17))
                .strafeToConstantHeading(new Vector2d(50, 32))
                .build();

        waitForStart();

        robot.setRetracted();

        Actions.runBlocking(toBackstage);
    }
}
