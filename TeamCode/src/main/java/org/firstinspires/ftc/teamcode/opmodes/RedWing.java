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
@Autonomous(name = "Red Wing (Park) \uD83E\uDD91", group = "red")
public class RedWing extends LinearOpMode {

    MecanumDrive drive;
    Robot robot;

    //Pose2d startPose = new Pose2d(-37, -62, Math.toRadians(270));
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));


    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);
        //robot = new Robot();
        //robot.init(hardwareMap, false);

        Action toBackstage = drive.actionBuilder(drive.pose)
//                .strafeToConstantHeading(new Vector2d(-37, -17))
//                .turnTo(Math.toRadians(180))
//                .strafeToConstantHeading(new Vector2d(50, -17))
//                .strafeToConstantHeading(new Vector2d(50, -32))
                .splineTo(new Vector2d(20, 20), Math.toRadians(90))
                .strafeTo(new Vector2d(0, 0))
                .build();

        waitForStart();

        //robot.setIntakeAngle(StandardTeleOp.INTAKE_UP);
        //robot.setRetracted();

        Actions.runBlocking(toBackstage);
    }
}
