package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.vision.PropCamera;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
@Autonomous(name = "New Red Backdrop \uD83E\uDD91", group = "red")
public class NewFullRedBackdrop extends LinearOpMode {

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

        //camera = new PropCamera(hardwareMap, telemetry, "Red", "Right");

        Action act11 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(12, -42), Math.toRadians(150))
                .build();

        Action act21 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(18, -38), Math.toRadians(90))
                .build();

        Action act31 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(23, -48), Math.toRadians(90))
                .build();

        Action act32 = drive.actionBuilder(new Pose2d(23, -48, Math.toRadians(90)))
                .strafeTo(new Vector2d(30, -48))
                .strafeToLinearHeading(new Vector2d(37, -41.5), 0)
                .build();

        Action act33 = drive.actionBuilder(new Pose2d(37, -41.5, 0))
                .strafeTo(new Vector2d(39, -41.5))
                .build();

        Action act34 = drive.actionBuilder(new Pose2d(39, -41.5, 0))
                .strafeTo(new Vector2d(37, -41.5))
                .strafeTo(new Vector2d(37, -10))
                .build();

        robot.setClawClosed();
        sleep(1000);
        robot.moveBase(0.7);
        robot.setCageDown();

        while (opModeInInit()) {
//            randomization = camera.getRandomization();
//            telemetry.addData("Randomization", randomization);
//            telemetry.update();
        }

        waitForStart();

        //camera.stopStreaming();

        telemetry.addData("Randomization", randomization);
        telemetry.update();

        Actions.runBlocking(act31);

        robot.moveBase(0.5);
        robot.moveWrist(0.05);
        robot.moveTop(0.7);
        sleep(1500);
        robot.setClawScoreOpen();
        sleep(1000);
        robot.setClawClosed();
        robot.setRetracted();
        sleep(1000);

        Actions.runBlocking(act32);

        drive.SETRAWDRIVEPOWERS(-0.3);
        sleep(500);
        drive.SETRAWDRIVEPOWERS(0);

        robot.setRetracted();
        sleep(1000);
        robot.setClawOpen();
        sleep(100);
        robot.setRetractedLowered();
        sleep(500);
        robot.setClawClosed();
        sleep(100);
        robot.setRetractedUp();
        sleep(1000);
        robot.moveBase(0.54);
        robot.moveTop(0.96);
        robot.moveWrist(0.025);
        sleep(1000);

        Actions.runBlocking(act33);

        robot.setClawOpen();
        sleep(500);

        Actions.runBlocking(act34);

        robot.setRetracted();
        sleep(1000);
    }
}
