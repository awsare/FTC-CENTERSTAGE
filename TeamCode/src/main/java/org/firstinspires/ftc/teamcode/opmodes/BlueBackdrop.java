package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.vision.PropCamera;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous
public class BlueBackdrop extends LinearOpMode {

    MecanumDrive drive;
    PropCamera camera;

    Pose2d startPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, startPose);
        camera = new PropCamera(hardwareMap, telemetry, "Blue");

//        Action act1 = drive.actionBuilder(drive.pose)
//                .splineTo(new Vector2d(30, 30), Math.PI / 2)
//                .splineTo(new Vector2d(60, 0), Math.PI)
//                .build();

        waitForStart();

//        Actions.runBlocking(act1);
    }

}
