package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp(name = "Operator Only \uD83C\uDFAE")
public class OperatorOnly extends LinearOpMode {

    public static double INTAKE_POWER = 0.7;
    public static double OUTTAKE_POWER = 0.7;

    public static double DRFB_UP_REDUCTION = 0.7;
    public static double DRFB_DOWN_REDUCTION = 0.005;
    public static double DRFB_GRAVITY = 0.125;

    enum ArmStates {
        RETRACTED_STATE,
        RETRACTED_LOWERED_STATE,
        SCORING_STATE,
        HOLDING
    }

    ArmStates armState;

    Gamepad previousOperator;
    Gamepad operator;

    ElapsedTime stateTime;
    ElapsedTime secondStateTime;
    ElapsedTime loopTime;

    Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot();
        robot.init(hardwareMap, true);
        robot.setClawClosed();
        robot.setLauncher();

        previousOperator = new Gamepad();
        operator = new Gamepad();

        armState = ArmStates.RETRACTED_STATE;

        stateTime = new ElapsedTime();
        secondStateTime = new ElapsedTime();
        loopTime = new ElapsedTime();

        waitForStart();

        gamepad1.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(255, 255, 255, Gamepad.LED_DURATION_CONTINUOUS);

        stateTime.reset();
        secondStateTime.reset();

        while (opModeIsActive()) {
            loopTime.reset();

            operatorControl();

            routineTasks();
            updateTelemetry();
        }
    }

    private void operatorControl() {
        if (operator.share) {
            DRFB_DOWN_REDUCTION = 1.0;
        }

        if (operator.square) {
            robot.setClawOpen();
        }

        if (operator.dpad_left) {
            robot.setClawClosed();
        }

        intakeControl();
        armControl();
        DRFBControl();
    }

    private void routineTasks() {
        previousOperator.copy(operator);
        operator.copy(gamepad2);
    }

    private void updateTelemetry() {
        telemetry.addData("DR4B Position", robot.getDRFBPosition());
        telemetry.addData("Arm State", armState.toString());
        telemetry.addData("Loop Time", Math.round(loopTime.time() * 1000));

        telemetry.update();
    }

    private void intakeControl() {
        if (operator.right_trigger > 0.1) {
            robot.powerIntake(-INTAKE_POWER);
        } else if (operator.left_trigger > 0.1) {
            robot.powerIntake(OUTTAKE_POWER);
        } else {
            robot.powerIntake(0);
        }
    }

    private void armControl() {
        switch (armState) {
            case RETRACTED_STATE:
                if (stateTime.time() < 0.2) {
                    break;
                }

                robot.setRetracted();

                if (operator.circle && !previousOperator.circle) {
                    armState = ArmStates.SCORING_STATE;

                    robot.setRetractedUp();

                    stateTime.reset();
                }

                if (operator.cross && !previousOperator.cross) {
                    armState = ArmStates.RETRACTED_LOWERED_STATE;
                    robot.setClawOpen();

                    secondStateTime.reset();
                    stateTime.reset();
                }

                if (operator.triangle && !previousOperator.triangle) {
                    stateTime.reset();
                    armState = ArmStates.HOLDING;
                }

                break;
            case RETRACTED_LOWERED_STATE:
                if (secondStateTime.time() < 0.2) {
                    stateTime.reset();
                    break;
                }

                robot.setRetractedLowered();

                if (stateTime.time() > 0.3) {
                    robot.setClawClosed();
                    stateTime.reset();
                    armState = ArmStates.RETRACTED_STATE;
                }

                break;
            case SCORING_STATE:
                if (stateTime.time() < 0.2) {
                    break;
                }

                robot.setScoring();

                if (operator.cross && !previousOperator.cross) {
                    armState = ArmStates.RETRACTED_STATE;

                    robot.setScoringIn();

                    stateTime.reset();
                }

                break;
            case HOLDING:
                robot.setClawWide();

                if (stateTime.time() < 0.2) {
                    break;
                }

                robot.setHolding();

                if (operator.triangle && !previousOperator.triangle) {
                    //robot.setClawOpen();
                    armState = ArmStates.RETRACTED_STATE;
                }
        }
    }

    private void DRFBControl() {
        double power = -operator.left_stick_y;

        if (robot.getDRFBPosition() > 1200 && power > 0) {
            power = 0;
        } else if (robot.getDRFBPosition() < -10 && power < 0 && !operator.left_bumper) {
            power = 0;
        }

        if (power > 0) {
            robot.powerDRFB(power * DRFB_UP_REDUCTION);
        } else if (power < 0) {
            robot.powerDRFB(power * DRFB_DOWN_REDUCTION);
        } else {
            if (robot.getDRFBPosition() < 100) {
                robot.powerDRFB(0);
            } else {
                robot.powerDRFB(DRFB_GRAVITY);
            }
        }
    }
}