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
    Servo baseLeft, baseRight, topLeft, topRight, wrist, claw, intakeAngle;

    public static class RETRACTED {
        public static double RETRACTED_BASE = 0.65;
        public static double RETRACTED_TOP = 0.2;
        public static double RETRACTED_WRIST = 0.1;
    }

    public static class RETRACTED_UP {
        public static double RETRACTED_UP_BASE = 0.54;
        public static double RETRACTED_UP_TOP = 0.3;
        public static double RETRACTED_UP_WRIST = 0.05;
    }

    public static class RETRACTED_LOWERED {
        public static double RETRACTED_LOWERED_BASE = 0.72;
        public static double RETRACTED_LOWERED_TOP = 0.225;
        public static double RETRACTED_LOWERED_WRIST = 0.05;
    }

    public static class SCORING {
        public static double SCORING_BASE = 0.3;
        public static double SCORING_TOP = 0.95;
        public static double SCORING_WRIST = 0.025;
    }

    public static class SCORING_LIFTED {
        public static double SCORING_LIFTED_BASE = 0.3;
        public static double SCORING_LIFTED_TOP = 1.0;
        public static double SCORING_LIFTED_WRIST = 0;
    }

    public static class GROUND {
        public static double GROUND_BASE = 0.5;
        public static double GROUND_TOP = 0.725;
        public static double GROUND_WRIST = 0.175;
    }

    public static class GROUND_LOWERED {
        public static double GROUND_LOWERED_BASE = 0.5;
        public static double GROUND_LOWERED_TOP = 0.725;
        public static double GROUND_LOWERED_WRIST = 0.175;
    }

    public void init(HardwareMap hardwareMap, boolean drive) {
        if (drive) {
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
        intakeAngle = hardwareMap.get(Servo.class, "intakeAngle");

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

    public void moveClaw(double position) {
        claw.setPosition(position);
    }

    public void powerDRFB(double power) {
        DRFBLeft.setPower(power);
        DRFBRight.setPower(power);
    }

    public int getDRFBPosition() {
        return DRFBRight.getCurrentPosition();
    }

    public void setRetracted() {
        moveBase(RETRACTED.RETRACTED_BASE);
        moveTop(RETRACTED.RETRACTED_TOP);
        moveWrist(RETRACTED.RETRACTED_WRIST);
    }

    public void setRetractedUp() {
        moveBase(RETRACTED_UP.RETRACTED_UP_BASE);
        moveTop(RETRACTED_UP.RETRACTED_UP_TOP);
        moveWrist(RETRACTED_UP.RETRACTED_UP_WRIST);
    }

    public void setRetractedLowered() {
        moveBase(RETRACTED_LOWERED.RETRACTED_LOWERED_BASE);
        moveTop(RETRACTED_LOWERED.RETRACTED_LOWERED_TOP);
        moveWrist(RETRACTED_LOWERED.RETRACTED_LOWERED_WRIST);
    }

    public void setScoring() {
        moveBase(SCORING.SCORING_BASE);
        moveTop(SCORING.SCORING_TOP);
        moveWrist(SCORING.SCORING_WRIST);
    }

    public void setScoringLifted() {
        moveBase(SCORING_LIFTED.SCORING_LIFTED_BASE);
        moveTop(SCORING_LIFTED.SCORING_LIFTED_TOP);
        moveWrist(SCORING_LIFTED.SCORING_LIFTED_WRIST);
    }

    public void setGround() {
        moveBase(GROUND.GROUND_BASE);
        moveTop(GROUND.GROUND_TOP);
        moveWrist(GROUND.GROUND_WRIST);
    }

    public void setGroundLowered() {
        moveBase(GROUND_LOWERED.GROUND_LOWERED_BASE);
        moveTop(GROUND_LOWERED.GROUND_LOWERED_TOP);
        moveWrist(GROUND_LOWERED.GROUND_LOWERED_WRIST);
    }
}
