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

    public static int centerXLeft = 310;
    public static int centerYLeft = 260;

    public static int centerXRight = 170;
    public static int centerYRight = 280;

    public static int outerXLeft = 0;
    public static int outerYLeft = 300;

    public static int outerXRight = 530;
    public static int outerYRight = 300;

    public static int boxSize = 65;

    public static int lowRedH = 0;
    public static int lowRedS = 100;
    public static int lowRedV = 50;

    public static int highRedH = 255;
    public static int highRedS = 255;
    public static int highRedV = 255;

    public static int lowBlueH = 0;
    public static int lowBlueS = 120;
    public static int lowBlueV = 20;

    public static int highBlueH = 255;
    public static int highBlueS = 255;
    public static int highBlueV = 255;

    Mat mat = new Mat();
    Mat center = new Mat();
    Mat outer = new Mat();

    Telemetry telemetry;

    String alliance;
    String side;

    int randomization;

    public PropDetector(Telemetry telemetry, String alliance, String side) {
        this.telemetry = telemetry;
        this.alliance = alliance;
        this.side = side;
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

        if (side.equals("Left")) {
            center = mat.submat(centerYLeft, centerYLeft + boxSize, centerXLeft, centerXLeft + boxSize);
            outer = mat.submat(outerYLeft, outerYLeft + boxSize, outerXLeft, outerXLeft + boxSize);
        } else {
            center = mat.submat(centerYRight, centerYRight + boxSize, centerXRight, centerXRight + boxSize);
            outer = mat.submat(outerYRight, outerYRight + boxSize, outerXRight, outerXRight + boxSize);
        }

        double centerAverage = Core.sumElems(center).val[0] / Math.pow(boxSize, 2) / 255.0;
        double outerAverage = Core.sumElems(outer).val[0] / Math.pow(boxSize, 2) / 255.0;

        if (side.equals("Left")) {
            Imgproc.rectangle(mat, new Rect(centerXLeft, centerYLeft, boxSize, boxSize), new Scalar(255, 255, 255), 3);
            Imgproc.rectangle(mat, new Rect(outerXLeft, outerYLeft, boxSize, boxSize), new Scalar(255, 255, 255), 3);
        } else {
            Imgproc.rectangle(mat, new Rect(centerXRight, centerYRight, boxSize, boxSize), new Scalar(255, 255, 255), 3);
            Imgproc.rectangle(mat, new Rect(outerXRight, outerYRight, boxSize, boxSize), new Scalar(255, 255, 255), 3);
        }

        center.release();
        outer.release();

        if (outerAverage > 0.25) {
            if (side.equals("Left")) {
                randomization = 0;
            } else {
                randomization = 2;
            }
        } else if (centerAverage > 0.25) {
            randomization = 1;
        } else {
            if (side.equals("Left")) {
                randomization = 2;
            } else {
                randomization = 0;
            }
        }

        telemetry.addData("Center %", centerAverage);
        telemetry.addData("Outer %", outerAverage);
        telemetry.addData("Randomization", randomization);
        telemetry.update();

        return mat;
    }

    public int getRandomization() {
        return randomization;
    }
}
