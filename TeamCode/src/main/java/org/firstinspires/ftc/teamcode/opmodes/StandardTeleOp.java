package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Robot;

@Config
@TeleOp
public class StandardTeleOp extends LinearOpMode {

    public static double LOW_SPEED = 0.325;
    public static double MEDIUM_SPEED = 0.7;
    public static double HIGH_SPEED = 1.0;
    public static double ROTATION_WEIGHT = 0.5;

    Gamepad previousDriver;
    Gamepad previousOperator;
    Gamepad driver;
    Gamepad operator;

    Robot robot;

    public static double kp = 4;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;
    public static int setPoint = 0;

    PIDFController pidfController = new PIDFController(kp, ki, kd, kf);

    DcMotorEx pidMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        previousDriver = new Gamepad();
        previousOperator = new Gamepad();
        driver = new Gamepad();
        operator = new Gamepad();

        //robot.init(hardwareMap);

        pidMotor = hardwareMap.get(DcMotorEx.class, "m");
        pidMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
//            driverControl();
//            operatorControl();
//            routineTasks();
//            updateTelemetry();

            pidfController.setSetPoint(setPoint);

            pidfController.setP(kp);
            pidfController.setI(ki);
            pidfController.setD(kd);
            pidfController.setF(kf);

            pidMotor.setVelocity(pidfController.calculate(pidMotor.getCurrentPosition()));

            dashboardTelemetry.addData("velo", pidMotor.getVelocity());
            dashboardTelemetry.addData("setpoint", setPoint);
            dashboardTelemetry.addData("pos", pidMotor.getCurrentPosition());

            dashboardTelemetry.update();

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

        robot.setDrivePowers(
            speed * ((y + x + r) / denominator),
            speed * ((y - x + r) / denominator),
            speed * ((y + x - r) / denominator),
            speed * ((y - x - r) / denominator)
        );
    }

    private void operatorControl() {

    }

    private void routineTasks() {
        previousDriver.copy(driver);
        previousOperator.copy(operator);
        driver.copy(gamepad1);
        operator.copy(gamepad2);
    }

    private void updateTelemetry() {

    }

}
