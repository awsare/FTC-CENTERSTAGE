package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;

@Config
@TeleOp
public class StandardTeleOp extends LinearOpMode {

    public static double LOW_SPEED = 0.325;
    public static double MEDIUM_SPEED = 0.7;
    public static double HIGH_SPEED = 1.0;
    public static double ROTATION_WEIGHT = 0.5;

    public static double RETRACT = 0.25;
    public static double RETRACT_LOWERED = 0.0;
    public static double SCORE = 1.0;
    public static double SCORE_LIFTED = 0.75;

    public enum ArmStates {
        RETRACT,
        RETRACT_LOWERED,
        SCORE,
        SCORE_LIFTED
    };

    public static ArmStates armState;

    Gamepad previousDriver;
    Gamepad previousOperator;
    Gamepad driver;
    Gamepad operator;

    ElapsedTime elapsedTime;

    Robot robot;
    ThreeDeadWheelLocalizer localizer;

    @Override
    public void runOpMode() {

        previousDriver = new Gamepad();
        previousOperator = new Gamepad();
        driver = new Gamepad();
        operator = new Gamepad();

        robot = new Robot();
        robot.init(hardwareMap);

        //localizer = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick);

        armState = ArmStates.RETRACT;

        elapsedTime = new ElapsedTime();

        waitForStart();

        elapsedTime.reset();

        while (opModeIsActive()) {
            driverControl();
//            operatorControl();
            routineTasks();
//            updateTelemetry();
        }
    }

    private void driverControl() {
        double speed = MEDIUM_SPEED;
        double change = driver.right_trigger - driver.left_trigger;

        speed += change * ((change > 0) ? HIGH_SPEED - MEDIUM_SPEED : MEDIUM_SPEED - LOW_SPEED);

        double r = driver.right_stick_x;
        double y = (r != 0) ? -driver.left_stick_y * (1 - ROTATION_WEIGHT) : -driver.left_stick_y;
        double x = (r != 0) ? driver.left_stick_x * (1 - ROTATION_WEIGHT) : driver.left_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);

//        double heading = localizer.
//
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        robot.setDrivePowers(
            speed * ((y + x + r) / denominator),
            speed * ((y - x + r) / denominator),
            speed * ((y + x - r) / denominator),
            speed * ((y - x - r) / denominator)
        );
    }

    private void operatorControl() {
        switch (armState) {
            case SCORE_LIFTED:
                robot.setTestServo(SCORE_LIFTED);

                break;
            case SCORE:
                robot.setTestServo(SCORE);

                break;
            case RETRACT:
                robot.setTestServo(RETRACT);

                break;
            case RETRACT_LOWERED:
                robot.setTestServo(RETRACT_LOWERED);

                break;
        }
    }

    private void routineTasks() {
        previousDriver.copy(driver);
        previousOperator.copy(operator);
        driver.copy(gamepad1);
        operator.copy(gamepad2);

        //localizer.update();
        robot.update();
    }

    private void updateTelemetry() {

    }

}
