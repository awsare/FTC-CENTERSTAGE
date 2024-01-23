package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Robot {

    DcMotorEx leftFront, leftBack, rightBack, rightFront, DRFBLeft, DRFBRight, intake;
    Servo baseLeft, baseRight, topLeft, topRight, wrist, claw, intakeLeft, intakeRight, cage, launcher;

    public static double RETRACTED_BASE = 0.8;
    public static double RETRACTED_TOP = 0.19;
    public static double RETRACTED_WRIST = 0.1;

    public static double RETRACTED_UP_BASE = 0.7;
    public static double RETRACTED_UP_TOP = 0.19;
    public static double RETRACTED_UP_WRIST = 0.1;

    public static double RETRACTED_LOWERED_BASE = 0.87;
    public static double RETRACTED_LOWERED_TOP = 0.19;
    public static double RETRACTED_LOWERED_WRIST = 0.05;

    public static double SCORING_BASE = 0.0;
    public static double SCORING_TOP = 1.0;
    public static double SCORING_WRIST = 0.07;

    public static double CLAW_OPEN = 0.3;
    public static double CLAW_SCORE_OPEN = 0.25;
    public static double CLAW_CLOSED = 0.4;

    public static double CAGE_UP = 1.0;
    public static double CAGE_DOWN = 0.65;

    public static double LAUNCHER_SET = 0.9;
    public static double LAUNCHER_SHOOT = 0.2;

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
        cage = hardwareMap.get(Servo.class, "cage");
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

    public void setCageUp() {
        cage.setPosition(CAGE_UP);
    }

    public void setCageDown() {
        cage.setPosition(CAGE_DOWN);
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

    public void setIntakeAngle(double angle) {
        intakeAngle.setPosition(angle);
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

    public void setClawClosed() {
        claw.setPosition(CLAW_CLOSED);
    }

    public void setClawOpen() {
        claw.setPosition(CLAW_OPEN);
    }

    public void setClawScoreOpen() {
        claw.setPosition(CLAW_SCORE_OPEN);
    }

    public void powerDRFB(double power) {
        DRFBLeft.setPower(power);
        DRFBRight.setPower(power);
    }

    public int getDRFBPosition() {
        return DRFBRight.getCurrentPosition();
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
        moveBase(0.5);
    }
}
