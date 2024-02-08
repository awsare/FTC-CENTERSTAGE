package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

@Config
@TeleOp(name = "Driver Only \uD83C\uDFAE")
public class DriverOnly extends LinearOpMode {

    public static double LOW_SPEED = 0.375;
    public static double MEDIUM_SPEED = 0.7;
    public static double HIGH_SPEED = 1.0;
    public static double ROTATION_WEIGHT = 0.5;

    Gamepad previousDriver;
    Gamepad driver;

    ElapsedTime loopTime;

    Robot robot;

    boolean inverseDrive = false;

    @Override
    public void runOpMode() {

        robot = new Robot();
        robot.init(hardwareMap, true);
        robot.setClawClosed();
        robot.setLauncher();

        previousDriver = new Gamepad();
        driver = new Gamepad();

        loopTime = new ElapsedTime();

        waitForStart();

        gamepad1.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(255, 255, 255, Gamepad.LED_DURATION_CONTINUOUS);

        while (opModeIsActive()) {
            loopTime.reset();

            driverControl();

            routineTasks();
            updateTelemetry();
        }
    }

    private void driverControl() {
        double speed = MEDIUM_SPEED;
        double change = driver.right_trigger - driver.left_trigger;

        speed += change * ((change > 0) ? HIGH_SPEED - MEDIUM_SPEED : MEDIUM_SPEED - LOW_SPEED);

        double r = driver.right_stick_x;
        double y = (r != 0) ? -driver.left_stick_y * (1 - ROTATION_WEIGHT) : -driver.left_stick_y;
        double x = (r != 0) ? driver.left_stick_x * (1 - ROTATION_WEIGHT) : driver.left_stick_x;

//        x = x * Math.cos(heading) - y * Math.sin(heading);
//        y = x * Math.sin(heading) + y * Math.cos(heading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);

        inverseDrive = driver.right_bumper;

        if (inverseDrive) {
            x = -x;
            y = -y;
        }

        robot.setDrivePowers(
                speed * ((y + x + r) / denominator),
                speed * ((y - x + r) / denominator),
                speed * ((y + x - r) / denominator),
                speed * ((y - x - r) / denominator)
        );

        if (driver.dpad_up && driver.triangle) {
            robot.shootLauncher();
        }

        if (driver.left_bumper) {
            robot.setIntakeDown();
        } else {
            robot.setIntakeUp();
        }
    }

    private void routineTasks() {
        previousDriver.copy(driver);
        driver.copy(gamepad1);
    }

    private void updateTelemetry() {
        telemetry.addData("Loop Time", Math.round(loopTime.time() * 1000));
        telemetry.update();
    }
}
