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

    public static int leftX = 80;
    public static int leftY = 110;

    public static int centerX = 335;
    public static int centerY = 90;

    public static int rightX = 555;
    public static int rightY = 100;

    public static int boxSize = 45;

    public static int lowRedH = 170;
    public static int lowRedS = 210;
    public static int lowRedV = 210;

    public static int highRedH = 190;
    public static int highRedS = 255;
    public static int highRedV = 255;

    public static int lowBlueH = 110;
    public static int lowBlueS = 100;
    public static int lowBlueV = 100;

    public static int highBlueH = 190;
    public static int highBlueS = 150;
    public static int highBlueV = 190;

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

        left = mat.submat(leftY, leftY + boxSize, leftX, leftX + boxSize);
        center = mat.submat(centerY, centerY + boxSize, centerX, centerX + boxSize);
        right = mat.submat(rightY, rightY + boxSize, rightX, rightX + boxSize);

        double leftAverage = Core.sumElems(left).val[0] / Math.pow(boxSize, 2) / 255.0;
        double centerAverage = Core.sumElems(center).val[0] / Math.pow(boxSize, 2) / 255.0;
        double rightAverage = Core.sumElems(right).val[0] / Math.pow(boxSize, 2) / 255.0;

        Imgproc.rectangle(mat, new Rect(leftX, leftY, boxSize, boxSize), new Scalar(255, 255, 255), 3);
        Imgproc.rectangle(mat, new Rect(centerX, centerY, boxSize, boxSize), new Scalar(255, 255, 255), 3);
        Imgproc.rectangle(mat, new Rect(rightX, rightY, boxSize, boxSize), new Scalar(255, 255, 255), 3);

        left.release();
        center.release();
        right.release();

        telemetry.addData("left %", leftAverage);
        telemetry.addData("center %", centerAverage);
        telemetry.addData("right %", rightAverage);
        telemetry.update();

        return mat;
    }

    public int getRandomization() {
        return randomization;
    }
}
