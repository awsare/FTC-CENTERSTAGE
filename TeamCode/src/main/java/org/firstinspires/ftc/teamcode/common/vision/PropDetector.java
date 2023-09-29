package org.firstinspires.ftc.teamcode.common.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PropDetector extends OpenCvPipeline {

    final int leftX = 100;
    final int leftY = 200;

    final int centerX = 275;
    final int centerY = 225;

    final int rightX = 505;
    final int rightY = 230;

    final int boxSize = 40;

    Scalar lowBlue = new Scalar(80, 210, 210);
    Scalar highBlue = new Scalar(200, 255, 255);

    Mat mat = new Mat();
    Mat left = new Mat();
    Mat center = new Mat();
    Mat right = new Mat();

    Telemetry telemetry;

    int randomization;

    public PropDetector(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        if (input.empty()) {
            return input;
        }

        input.copyTo(mat);

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(mat, lowBlue, highBlue, mat);

        left = mat.submat(leftY, leftY + boxSize, leftX, leftX + boxSize);
        center = mat.submat(centerY, centerY + boxSize, centerX, centerX + boxSize);
        right = mat.submat(rightY, rightY + boxSize, rightX, rightX + boxSize);

        Imgproc.rectangle(mat, new Rect(leftX, leftY, boxSize, boxSize), new Scalar(255, 255, 255), 3);
        Imgproc.rectangle(mat, new Rect(centerX, centerY, boxSize, boxSize), new Scalar(255, 255, 255), 3);
        Imgproc.rectangle(mat, new Rect(rightX, rightY, boxSize, boxSize), new Scalar(255, 255, 255), 3);

        double leftAverage = Core.sumElems(left).val[0] / Math.pow(boxSize, 2) / 255.0;
        double centerAverage = Core.sumElems(center).val[0] / Math.pow(boxSize, 2) / 255.0;
        double rightAverage = Core.sumElems(right).val[0] / Math.pow(boxSize, 2) / 255.0;

//        double leftTotal = 0;
//        for (int row = 0; row < boxSize; row++) {
//            for (int col = 0; col < boxSize; col++) {
//                double[] pixel = left.get(row, col);
//                leftTotal += pixel[0];
//            }
//        }
//
//        double centerTotal = 0;
//        for (int row = 0; row < boxSize; row++) {
//            for (int col = 0; col < boxSize; col++) {
//                double[] pixel = center.get(row, col);
//                centerTotal += pixel[0];
//            }
//        }
//
//        double rightTotal = 0;
//        for (int row = 0; row < boxSize; row++) {
//            for (int col = 0; col < boxSize; col++) {
//                double[] pixel = right.get(row, col);
//                rightTotal += pixel[0];
//            }
//        }

        left.release();
        center.release();
        right.release();

//        double leftAverage = (leftTotal / (double)(boxSize * boxSize));
//        double centerAverage = (centerTotal / (double)(boxSize * boxSize));
//        double rightAverage = (rightTotal / (double)(boxSize * boxSize));

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
