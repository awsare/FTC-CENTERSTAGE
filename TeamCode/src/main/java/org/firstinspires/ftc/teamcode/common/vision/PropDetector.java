package org.firstinspires.ftc.teamcode.common.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class PropDetector extends OpenCvPipeline {

    public static int leftXRed = 80;
    public static int leftYRed = 280;

    public static int centerXRed = 335;
    public static int centerYRed = 300;

    public static int rightXRed = 555;
    public static int rightYRed = 270;

    public static int leftXBlue = 130;
    public static int leftYBlue = 290;

    public static int centerXBlue = 335;
    public static int centerYBlue = 300;

    public static int rightXBlue = 555;
    public static int rightYBlue = 280;

    public static int boxSize = 55;

    public static int lowRedH = 0;
    public static int lowRedS = 120;
    public static int lowRedV = 50;

    public static int highRedH = 255;
    public static int highRedS = 255;
    public static int highRedV = 255;

    public static int lowBlueH = 0;
    public static int lowBlueS = 100;
    public static int lowBlueV = 0;

    public static int highBlueH = 255;
    public static int highBlueS = 255;
    public static int highBlueV = 255;

    Mat mat = new Mat();
    Mat left = new Mat();
    Mat center = new Mat();
    Mat right = new Mat();

    Telemetry telemetry;

    public static String alliance = "red";

    int randomization;

    public PropDetector(Telemetry telemetry, String alliance) {
        this.telemetry = telemetry;
        this.alliance = alliance;
    }

    @Override
    public Mat processFrame(Mat input) {
        if (input.empty()) {
            return input;
        }

        input.copyTo(mat);

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);

        if (alliance.equals("Red")) {
            Core.inRange(mat, new Scalar(lowRedH, lowRedS, lowRedV), new Scalar(highRedH, highRedS, highRedV), mat);
        } else {
            Core.inRange(mat, new Scalar(lowBlueH, lowBlueS, lowBlueV), new Scalar(highBlueH, highBlueS, highBlueV), mat);
        }

        if (alliance.equals("Red")) {
            left = mat.submat(leftYRed, leftYRed + boxSize, leftXRed, leftXRed + boxSize);
            center = mat.submat(centerYRed, centerYRed + boxSize, centerXRed, centerXRed + boxSize);
            right = mat.submat(rightYRed, rightYRed + boxSize, rightXRed, rightXRed + boxSize);
        } else {
            left = mat.submat(leftYBlue, leftYBlue + boxSize, leftXBlue, leftXBlue + boxSize);
            center = mat.submat(centerYBlue, centerYBlue + boxSize, centerXBlue, centerXBlue + boxSize);
            right = mat.submat(rightYBlue, rightYBlue + boxSize, rightXBlue, rightXBlue + boxSize);
        }

        double leftAverage = Core.sumElems(left).val[0] / Math.pow(boxSize, 2) / 255.0;
        double centerAverage = Core.sumElems(center).val[0] / Math.pow(boxSize, 2) / 255.0;
        double rightAverage = Core.sumElems(right).val[0] / Math.pow(boxSize, 2) / 255.0;

        if (alliance.equals("Red")) {
            Imgproc.rectangle(mat, new Rect(leftXRed, leftYRed, boxSize, boxSize), new Scalar(255, 255, 255), 3);
            Imgproc.rectangle(mat, new Rect(centerXRed, centerYRed, boxSize, boxSize), new Scalar(255, 255, 255), 3);
            Imgproc.rectangle(mat, new Rect(rightXRed, rightYRed, boxSize, boxSize), new Scalar(255, 255, 255), 3);
        } else {
            Imgproc.rectangle(mat, new Rect(leftXBlue, leftYBlue, boxSize, boxSize), new Scalar(255, 255, 255), 3);
            Imgproc.rectangle(mat, new Rect(centerXBlue, centerYBlue, boxSize, boxSize), new Scalar(255, 255, 255), 3);
            Imgproc.rectangle(mat, new Rect(rightXBlue, rightYBlue, boxSize, boxSize), new Scalar(255, 255, 255), 3);
        }

        left.release();
        center.release();
        right.release();

        telemetry.addData("left %", leftAverage);
        telemetry.addData("center %", centerAverage);
        telemetry.addData("right %", rightAverage);
        telemetry.update();

        if (leftAverage > centerAverage && leftAverage > rightAverage) {
            randomization = 0;
        } else if (centerAverage > leftAverage && centerAverage > rightAverage) {
            randomization = 1;
        } else if (rightAverage > leftAverage && rightAverage > centerAverage) {
            randomization = 2;
        } else {
            randomization = 0;
        }

        return mat;
    }

    public int getRandomization() {
        return randomization;
    }
}
