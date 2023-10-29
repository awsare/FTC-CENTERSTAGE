package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;

public class BlueWing extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot();
        robot.init(hardwareMap, telemetry);

        waitForStart();

        robot.setDrivePowers(0.5, -0.5, 0.5, -0.5);
        sleep(1000);
        robot.setDrivePowers(0, 0, 0, 0);
        robot.setScoring();
        sleep(500);
    }
}
