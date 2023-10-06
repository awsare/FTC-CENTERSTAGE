package org.firstinspires.ftc.teamcode.common;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    DcMotorEx leftFront, leftBack, rightBack, rightFront, DRFBLeft, DRFBRight, intake, launcher;

    PIDFController DRFBPIDF = new PIDFController(0, 0, 0, 0);
    PIDFController intakePIDF = new PIDFController(0, 0, 0, 0);
    PIDFController launcherPIDF = new PIDFController(0, 0, 0, 0);

    public void init(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DRFBLeft = hardwareMap.get(DcMotorEx.class, "DRFBLeft");
        DRFBRight = hardwareMap.get(DcMotorEx.class, "DRFBRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
    }

    public void setDrivePowers(double leftFront, double leftBack, double rightBack, double rightFront) {
        this.leftFront.setPower(leftFront);
        this.leftBack.setPower(leftBack);
        this.rightBack.setPower(rightBack);
        this.rightFront.setPower(rightFront);
    }
}
