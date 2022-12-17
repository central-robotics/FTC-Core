package com.chsrobotics.ftccore.vision;

import com.chsrobotics.ftccore.hardware.HardwareManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class CVUtility {

    private volatile OpenCvCamera camera;
    private volatile InterfacePipeline pipeline;
    public volatile boolean initialized = false;
    private static Telemetry telem;
    private Thread thread;

    public CVUtility(HardwareManager manager, Telemetry telemetry)
    {
        telem = telemetry;
        thread = new Thread(() -> {
            camera = OpenCvCameraFactory.getInstance().createWebcam(manager.accessoryCameras[0]);

            pipeline = new InterfacePipeline();
            camera.setPipeline(pipeline);

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    initialized = true;
                    startStreaming();
                }

                @Override
                public void onError(int errorCode) {
                    manager.useCV = false;
                    telemetry.addData("OpenCV error", errorCode);
                }
            });
        });
        thread.start();
    }

    public void startStreaming()
    {
        camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
    }

    public Mat grabFrame()
    {
        return pipeline.latestFrame;
    }

    public void stopStreaming()
    {
        if (thread.isAlive()) {
            try {
                thread.interrupt();
            } catch (Exception e) {

            }
        }
        camera.stopStreaming();
    }

    public class InterfacePipeline extends OpenCvPipeline
    {
        public Mat latestFrame;

        @Override
        public Mat processFrame(Mat input) {
            if (latestFrame == null) {
                telem.addLine("Webcam ready");
                telem.update();
            }
            latestFrame = input;
            return null;
        }
    }
}
