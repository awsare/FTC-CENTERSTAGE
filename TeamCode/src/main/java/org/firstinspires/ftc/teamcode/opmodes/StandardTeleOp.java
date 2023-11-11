package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

@Config
@TeleOp
public class StandardTeleOp extends LinearOpMode {

    public static double LOW_SPEED = 0.375;
    public static double MEDIUM_SPEED = 0.7;
    public static double HIGH_SPEED = 1.0;
    public static double ROTATION_WEIGHT = 0.5;

    public static double CLAW_OPEN = 0.3;
    public static double CLAW_SCORE_OPEN = 0.2;
    public static double CLAW_CLOSED = 0.4;

    public static double INTAKE_UP = 0.2;
    public static double INTAKE_HIGH_UP = 0.1;
    public static double INTAKE_DOWN = 0.45;

    public static double INTAKE_POWER = 0.5;

    public static double DRFB_UP_REDUCTION = 0.5;
    public static double DRFB_DOWN_REDUCTION = 0.05;

    double heading;

    public enum ArmStates {
        RETRACTED_STATE,
        RETRACTED_LOWERED_STATE,
        SCORING_STATE,
        SCORING_LIFTED_STATE,
        GROUND_STATE,
        GROUND_LOWERED_STATE
    };

    public static ArmStates armState;

    Gamepad previousDriver;
    Gamepad previousOperator;
    Gamepad driver;
    Gamepad operator;

    ElapsedTime stateTime;
    ElapsedTime secondStateTime;
    ElapsedTime loopTime;
    ElapsedTime matchTime;

    Robot robot;

    @Override
    public void runOpMode() {

        previousDriver = new Gamepad();
        previousOperator = new Gamepad();
        driver = new Gamepad();
        operator = new Gamepad();

        robot = new Robot();
        robot.init(hardwareMap, telemetry, true);

        armState = ArmStates.RETRACTED_STATE;

        stateTime = new ElapsedTime();
        secondStateTime = new ElapsedTime();
        loopTime = new ElapsedTime();
        matchTime = new ElapsedTime();

        robot.moveClaw(CLAW_CLOSED);

        waitForStart();

        stateTime.reset();
        secondStateTime.reset();
        matchTime.reset();

        while (opModeIsActive()) {
            loopTime.reset();

            driverControl();
            operatorControl();

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

        robot.setDrivePowers(
            speed * ((y + x + r) / denominator),
            speed * ((y - x + r) / denominator),
            speed * ((y + x - r) / denominator),
            speed * ((y - x - r) / denominator)
        );
    }

    private void operatorControl() {

        if (operator.share) {
            DRFB_DOWN_REDUCTION = 1.0;
            robot.setIntakeAngle(INTAKE_HIGH_UP);
        }

        if (operator.dpad_up) {
            robot.setIntakeAngle(INTAKE_UP);
        } else if (operator.dpad_down) {
            robot.setIntakeAngle(INTAKE_DOWN);
        }

        if (operator.right_trigger > 0.1) {
            robot.powerIntake(-INTAKE_POWER);
        } else if (operator.left_trigger > 0.1) {
            robot.powerIntake(INTAKE_POWER);
        } else {
            robot.powerIntake(0);
        }

        if (operator.x && !previousOperator.x) {
            robot.moveClaw(CLAW_OPEN);
        }

        switch (armState) {
            case RETRACTED_STATE:
                robot.setRetracted();

                if (operator.b && !previousOperator.b) {
                    armState = ArmStates.SCORING_STATE;

                    robot.setRetractedUp();

                    stateTime.reset();
                }

                if (operator.y && !previousOperator.y) {
                    armState = ArmStates.GROUND_STATE;
                    robot.moveClaw(CLAW_OPEN);
                }

                if (operator.a && !previousOperator.a) {
                    armState = ArmStates.RETRACTED_LOWERED_STATE;
                    robot.moveClaw(CLAW_OPEN);

                    secondStateTime.reset();
                    stateTime.reset();
                }

                break;
            case RETRACTED_LOWERED_STATE:
                if (secondStateTime.time() < 0.25) {
                    stateTime.reset();
                    break;
                }

                robot.setRetractedLowered();

                if (stateTime.time() > 0.5) {
                    robot.moveClaw(CLAW_CLOSED);
                    armState = ArmStates.RETRACTED_STATE;
                }

                break;
            case SCORING_STATE:
                if (stateTime.time() < 0.5) {
                    break;
                }

                robot.setScoring();

                if (operator.a && !previousOperator.a) {
                    armState = ArmStates.RETRACTED_STATE;
                }

                if (operator.y && !previousOperator.y) {
                    armState = ArmStates.GROUND_STATE;
                }

                if (operator.b && !previousOperator.b) {
                    armState = ArmStates.SCORING_LIFTED_STATE;
                    robot.moveClaw(CLAW_SCORE_OPEN);

                    stateTime.reset();
                }

                break;
            case SCORING_LIFTED_STATE:
                robot.setScoringLifted();

                if (stateTime.time() > 0.5) {
                    robot.moveClaw(CLAW_OPEN);
                    armState = ArmStates.SCORING_STATE;
                }

                break;
            case GROUND_STATE:
                robot.setGround();

                if (operator.a && !previousOperator.a) {
                    armState = ArmStates.RETRACTED_STATE;
                }

                if (operator.b && !previousOperator.b) {
                    armState = ArmStates.SCORING_STATE;
                }

                if (operator.y && !previousOperator.y) {
                    armState = ArmStates.GROUND_LOWERED_STATE;
                    robot.moveClaw(CLAW_OPEN);

                    stateTime.reset();
                }

                break;
            case GROUND_LOWERED_STATE:
                robot.setGroundLowered();

                if (stateTime.time() > 0.5) {
                    robot.moveClaw(CLAW_CLOSED);
                    armState = ArmStates.GROUND_STATE;
                }

                break;
        }

        double power = -operator.left_stick_y;

        if (robot.getDRFBPosition() > 1200 && power > 0) {
            power = 0;
        } else if (robot.getDRFBPosition() < -20 && power < 0) {
            power = 0;
        }

        if (power > 0) {
            robot.powerDRFB(power * DRFB_UP_REDUCTION);
        } else if (power < 0) {
            robot.powerDRFB(power * DRFB_DOWN_REDUCTION);
        } else {
            robot.powerDRFB(0);
        }
    }

    private void routineTasks() {
        previousDriver.copy(driver);
        previousOperator.copy(operator);
        driver.copy(gamepad1);
        operator.copy(gamepad2);
    }

    private void updateTelemetry() {
        telemetry.addData("DR4B Position", robot.getDRFBPosition());
        telemetry.addData("Arm State", armState.toString());
        telemetry.addData("Loop Time", Math.round(loopTime.time() * 1000));
        telemetry.addData("Match Time", Math.round(matchTime.seconds()));

        telemetry.update();
    }

}
