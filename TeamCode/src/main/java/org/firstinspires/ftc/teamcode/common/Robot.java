package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Robot {

    DcMotorEx leftFront, leftBack, rightBack, rightFront, DRFBLeft, DRFBRight, intake/*, launcher*/;
    Servo baseLeft, baseRight, topLeft, topRight, wrist, claw, intakeAngle;

    IMU imu;

    Telemetry dashboardTelemetry;
    Telemetry hubTelemetry;

    public static double RETRACTED_BASE = 1.0;
    public static double RETRACTED_TOP = 0.3;
    public static double RETRACTED_WRIST = 0.05;

    public static double RETRACTED_UP_TOP = 0.55;
    public static double RETRACTED_UP_WRIST = 0.1;

    public static double RETRACTED_LOWERED_BASE = 1.0;
    public static double RETRACTED_LOWERED_TOP = 0.2;
    public static double RETRACTED_LOWERED_WRIST = 0.15;

    public static double SCORING_BASE = 0.6;
    public static double SCORING_TOP = 0.9;
    public static double SCORING_WRIST = 0.175;

    public static double SCORING_LIFTED_BASE = 0.6;
    public static double SCORING_LIFTED_TOP = .825;
    public static double SCORING_LIFTED_WRIST = 0.175;

    public static double GROUND_BASE = 0.5;
    public static double GROUND_TOP = 0.7;
    public static double GROUND_WRIST = 0.175;

    public static double GROUND_LOWERED_BASE = .48;
    public static double GROUND_LOWERED_TOP = .635;
    public static double GROUND_LOWERED_WRIST = .23;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DRFBLeft = hardwareMap.get(DcMotorEx.class, "DRFBLeft");
        DRFBRight = hardwareMap.get(DcMotorEx.class, "DRFBRight");
        //intake = hardwareMap.get(DcMotorEx.class, "intake");

        baseLeft = hardwareMap.get(Servo.class, "baseLeft");
        baseRight = hardwareMap.get(Servo.class, "baseRight");
        topLeft = hardwareMap.get(Servo.class, "topLeft");
        topRight = hardwareMap.get(Servo.class, "topRight");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        //intakeAngle = hardwareMap.get(Servo.class, "intakeAngle");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP)
        ));

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        DRFBRight.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DRFBLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DRFBRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        hubTelemetry = telemetry;
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

    public void setIntakePosition(double position) {
        intakeAngle.setPosition(position);
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

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetIMUYaw() {
        imu.resetYaw();
    }

    public void setRetracted() {
        moveBase(RETRACTED_BASE);
        moveTop(RETRACTED_TOP);
        moveWrist(RETRACTED_WRIST);
    }

    public void setRetractedUp() {
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
