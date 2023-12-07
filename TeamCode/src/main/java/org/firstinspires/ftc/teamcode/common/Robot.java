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

    //public static class RETRACTED {
        public static double RETRACTED_BASE = 0.875;
        public static double RETRACTED_TOP = 0.19;
        public static double RETRACTED_WRIST = 0.1;
    //}

    //public static class RETRACTED_UP {
        public static double RETRACTED_UP_BASE = 0.54;
        public static double RETRACTED_UP_TOP = 0.24;
        public static double RETRACTED_UP_WRIST = 0.05;
    //}

    //public static class RETRACTED_LOWERED {
        public static double RETRACTED_LOWERED_BASE = 0.95;
        public static double RETRACTED_LOWERED_TOP = 0.19;
        public static double RETRACTED_LOWERED_WRIST = 0.05;
    //}

    //public static class SCORING {
        public static double SCORING_BASE = 0.54;
        public static double SCORING_TOP = 0.96;
        public static double SCORING_WRIST = 0.025;
    //}

    //public static class SCORING_LIFTED {
        public static double SCORING_LIFTED_BASE = 0.54;
        public static double SCORING_LIFTED_TOP = 1.0;
        public static double SCORING_LIFTED_WRIST = 0;
    //}

    public static double FAR_SCORING_BASE = 0.55;
    public static double FAR_SCORING_TOP = 0.96;
    public static double FAR_SCORING_WRIST = 0.025;

    public static double FAR_SCORING_LIFTED_BASE = 0.55;
    public static double FAR_SCORING_LIFTED_TOP = 0.96;
    public static double FAR_SCORING_LIFTED_WRIST = 0.025;

    //public static class GROUND {
        public static double GROUND_BASE = 0.5;
        public static double GROUND_TOP = 0.725;
        public static double GROUND_WRIST = 0.175;
    //}

    //public static class GROUND_LOWERED {
        public static double GROUND_LOWERED_BASE = 0.5;
        public static double GROUND_LOWERED_TOP = 0.725;
        public static double GROUND_LOWERED_WRIST = 0.175;
    //}

    //public static class CLAW {
        public static double CLAW_OPEN = 0.3;
        public static double CLAW_SCORE_OPEN = 0.25;
        public static double CLAW_CLOSED = 0.4;
    //}

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

    public void setScoringLifted() {
        moveBase(SCORING_LIFTED_BASE);
        moveTop(SCORING_LIFTED_TOP);
        moveWrist(SCORING_LIFTED_WRIST);
    }

    public void setFarScoring() {
        moveBase(FAR_SCORING_BASE);
        moveTop(FAR_SCORING_TOP);
        moveWrist(FAR_SCORING_WRIST);
    }

    public void setFarScoringLifted() {
        moveBase(FAR_SCORING_LIFTED_BASE);
        moveTop(FAR_SCORING_LIFTED_TOP);
        moveWrist(FAR_SCORING_LIFTED_WRIST);
    }

    public void setGround() {
        moveBase(GROUND_BASE);
        moveTop(GROUND_TOP);
        moveWrist(GROUND_WRIST);
    }

    public void setGroundLowered() {
        moveBase(GROUND_LOWERED_BASE);
        moveTop(GROUND_LOWERED_TOP);
        moveWrist(GROUND_LOWERED_WRIST);
    }
}
