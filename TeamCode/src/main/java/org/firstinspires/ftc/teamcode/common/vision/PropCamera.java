package org.firstinspires.ftc.teamcode.common.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class PropCamera {

    PropDetector propDetector;
    OpenCvCamera camera;

    public PropCamera(HardwareMap hardwareMap, Telemetry telemetry, String alliance, String side) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "propCamera"), cameraMonitorViewId);
        propDetector = new PropDetector(telemetry, alliance, side);

        camera.setPipeline(propDetector);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 30);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public int getRandomization() {
        return propDetector.getRandomization();
    }

    public void stopStreaming() {
        camera.stopStreaming();
    }
}
