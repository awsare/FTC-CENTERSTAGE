package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Robot {

    DcMotorEx leftFront, leftBack, rightBack, rightFront, DRFBLeft, DRFBRight, intake, launcher;

    Servo testServo;

    PIDFController DRFBPIDF = new PIDFController(0, 0, 0, 0);
    PIDFController intakePIDF = new PIDFController(0, 0, 0, 0);
    PIDFController launcherPIDF = new PIDFController(0, 0, 0, 0);

    Telemetry dashboardTelemetry;

    public static double kp = 4;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;
    public static int setPoint = 0;
    PIDFController pidfController = new PIDFController(kp, ki, kd, kf);
    DcMotorEx pidMotor;

    public void init(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
//        DRFBLeft = hardwareMap.get(DcMotorEx.class, "DRFBLeft");
//        DRFBRight = hardwareMap.get(DcMotorEx.class, "DRFBRight");
//        intake = hardwareMap.get(DcMotorEx.class, "intake");
//        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

//        pidMotor = hardwareMap.get(DcMotorEx.class, "m");
//        pidMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

        //testServo = hardwareMap.get(Servo.class, "testServo");
    }

    public void setDrivePowers(double leftFront, double leftBack, double rightBack, double rightFront) {
        this.leftFront.setPower(leftFront);
        this.leftBack.setPower(leftBack);
        this.rightBack.setPower(rightBack);
        this.rightFront.setPower(rightFront);
    }

    public void update() {
//        pidfController.setSetPoint(setPoint);
//
//        pidfController.setP(kp);
//        pidfController.setI(ki);
//        pidfController.setD(kd);
//        pidfController.setF(kf);
//
//        pidMotor.setVelocity(pidfController.calculate(pidMotor.getCurrentPosition()));
//
//        dashboardTelemetry.addData("velocity", pidMotor.getVelocity());
//        dashboardTelemetry.addData("setpoint", setPoint);
//        dashboardTelemetry.addData("position", pidMotor.getCurrentPosition());
//        dashboardTelemetry.update();
    }

    public void setTestServo(double pos) {
        testServo.setPosition(pos);
    }

    public double getTestServo() {
        return testServo.getPosition();
    }
}
