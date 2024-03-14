package org.firstinspires.ftc.teamcode.common;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ActionRobot {

    DcMotorEx leftFront, leftBack, rightBack, rightFront, DRFBLeft, DRFBRight, intake;
    Servo baseLeft, baseRight, topLeft, topRight, wrist, claw, intakeLeft, intakeRight, launcher;

    public static double RETRACTED_BASE = 0.65;
    public static double RETRACTED_TOP = 0.17;
    public static double RETRACTED_WRIST = 0.325;

    public static double RETRACTED_UP_BASE = 0.5;
    public static double RETRACTED_UP_TOP = 0.19;
    public static double RETRACTED_UP_WRIST = 0.325;

    public static double RETRACTED_LOWERED_BASE = 0.7;
    public static double RETRACTED_LOWERED_TOP = 0.11;
    public static double RETRACTED_LOWERED_WRIST = 0.41;

    public static double SCORING_BASE = 0.0;
    public static double SCORING_TOP = 0.99;
    public static double SCORING_WRIST = 0.225;

    public static double SCORING_IN_BASE = 0.5;
    public static double SCORING_IN_TOP = 1.0;
    public static double SCORING_IN_WRIST = 0.225;

    public static double CLAW_OPEN = 0.395;
    public static double CLAW_SCORE_OPEN = 0.34;
    public static double CLAW_CLOSED = 0.485;

    public static double INTAKE_UP = 0.64;
    public static double INTAKE_DOWN = 0.6625;
    public static double INTAKE_SERVO_OFFSET = 0.18;

    public static double LAUNCHER_SET = 1.0;
    public static double LAUNCHER_SHOOT = 0.0;

    public void init(HardwareMap hardwareMap, boolean teleop) {
        if (teleop) {
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        DRFBLeft = hardwareMap.get(DcMotorEx.class, "DRFBLeft");
        DRFBRight = hardwareMap.get(DcMotorEx.class, "DRFBRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        baseLeft = hardwareMap.get(Servo.class, "baseLeft");
        baseRight = hardwareMap.get(Servo.class, "baseRight");
        topLeft = hardwareMap.get(Servo.class, "topLeft");
        topRight = hardwareMap.get(Servo.class, "topRight");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        launcher = hardwareMap.get(Servo.class, "launcher");

        DRFBRight.setDirection(DcMotorSimple.Direction.REVERSE);

        DRFBLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DRFBRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setDrivePowers(double leftFront, double leftBack, double rightBack, double rightFront) {
        this.leftFront.setPower(leftFront);
        this.leftBack.setPower(leftBack);
        this.rightBack.setPower(rightBack);
        this.rightFront.setPower(rightFront);
    }

    public void setLauncher() {
        launcher.setPosition(LAUNCHER_SET);
    }

    public void shootLauncher() {
        launcher.setPosition(LAUNCHER_SHOOT);
    }

    public void powerIntake(double power) {
        intake.setPower(power);
    }

    public void setIntakeUp() {
        intakeLeft.setPosition(INTAKE_UP);
        intakeRight.setPosition(1 - (INTAKE_UP - INTAKE_SERVO_OFFSET));
    }

    public void setIntakeDown() {
        intakeLeft.setPosition(INTAKE_DOWN);
        intakeRight.setPosition(1 - (INTAKE_DOWN - INTAKE_SERVO_OFFSET));
    }

    public void powerDRFB(double power) {
        DRFBLeft.setPower(power);
        DRFBRight.setPower(power);
    }

    public int getDRFBPosition() {
        return DRFBRight.getCurrentPosition();
    }

    public void liftToAutoHeight() {
        while (getDRFBPosition() < 200) {
            powerDRFB(0.7);
        }
    }

    public void lower() {
        while (getDRFBPosition() > 100) {
            powerDRFB(-0.05);
        }
    }

    public void setClawClosed() {
        claw.setPosition(CLAW_CLOSED);
    }

    public void setClawOpen() {
        claw.setPosition(CLAW_OPEN);
    }

    public void moveBase(double position) {
        baseLeft.setPosition(position);
        baseRight.setPosition(1 - position);
    }

    public void moveTop(double position) {
        topLeft.setPosition(position);
        topRight.setPosition(1 - position);
    }

    public void moveWrist(double position) {
        wrist.setPosition(position);
    }

    public void setClawScoreOpen() {
        claw.setPosition(CLAW_SCORE_OPEN);
    }

    public void setRetracted() {
        moveBase(RETRACTED_BASE);
        moveTop(RETRACTED_TOP);
        moveWrist(RETRACTED_WRIST);
    }

    public void setRetractedUp() {
        moveBase(RETRACTED_UP_BASE);
        moveTop(RETRACTED_UP_TOP);
        moveWrist(RETRACTED_UP_WRIST);
    }

    public void setRetractedLowered() {
        moveBase(RETRACTED_LOWERED_BASE);
        moveTop(RETRACTED_LOWERED_TOP);
        moveWrist(RETRACTED_LOWERED_WRIST);
    }

    public void setScoring() {
        moveBase(SCORING_BASE);
        moveTop(SCORING_TOP);
        moveWrist(SCORING_WRIST);
    }

    public void setScoringIn() {
        moveBase(SCORING_IN_BASE);
        moveTop(SCORING_IN_TOP);
        moveWrist(SCORING_IN_WRIST);
    }

    private class Transfer implements Action {
        boolean init = false;

        int state = 0;
        ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                timer.reset();
                init = true;
            }

            switch (state) {
                case 0:
                    setClawOpen();

                    if (timer.time() < 0.225) {
                        break;
                    }

                    setRetractedLowered();

                    timer.reset();
                    state = 1;

                    break;
                case 1:
                    if (timer.time() < 0.45) {
                        break;
                    }

                    setClawClosed();
                    timer.reset();
                    state = 2;
                case 2:
                    if (timer.time() < 0.2) {
                        break;
                    }

                    setRetracted();
                    return false;
            }

            return true;
        }
    }
    public Action transfer() {
        return new Transfer();
    }

    private class Deposit implements Action {
        boolean init = false;

        ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                timer.reset();
                init = true;
            }

            if (timer.time() < 0.5) {
                setRetractedUp();
            } else {
                moveTop(0.65);
                moveWrist(0.5);
                moveBase(0.14);
                return false;
            }

            return true;
        }
    }
    public Action deposit() {
        return new Deposit();
    }
}
