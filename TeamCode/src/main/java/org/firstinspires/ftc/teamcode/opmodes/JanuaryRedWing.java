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
@Autonomous(name = "Jan Red Wing \uD83E\uDD91", group = "red")
public class JanuaryRedWing extends LinearOpMode {

    MecanumDrive drive;
    Robot robot;
    PropCamera camera;

    int randomization = 0;

    Pose2d startPose = new Pose2d(-36, -62, Math.PI / 2.0);

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, startPose);
        robot = new Robot();
        robot.init(hardwareMap, false);

        //camera = new PropCamera(hardwareMap, telemetry, "Red", "Left");

        Action run = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(-50, -44))
                .strafeToConstantHeading(new Vector2d(-50, -6))
                .strafeToConstantHeading(new Vector2d(-35, -10))
                .turnTo(Math.toRadians(-110))
                .waitSeconds(1)
                .turnTo(Math.toRadians(5))
                .strafeToConstantHeading(new Vector2d(-35, -7))
                .waitSeconds(3)
                .strafeToConstantHeading(new Vector2d(40, -7))
                .strafeToConstantHeading(new Vector2d(40, -29))
                .waitSeconds(3)
                .strafeToConstantHeading(new Vector2d(42, -29))
                .strafeToConstantHeading(new Vector2d(40, -29))
                .build();

        robot.setClawClosed();
        sleep(1000);
        robot.moveBase(0.7);
        robot.setCageDown();

//        while (opModeInInit()) {
//            randomization = camera.getRandomization();
//            telemetry.addData("Randomization", randomization);
//            telemetry.update();
//        }

        waitForStart();

        Actions.runBlocking(run);

    }
}
