package com.chsrobotics.ftccore.utilities;

import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class ComputerVision {
    private static OpenCvWebcam webcam;
    private static OpenCvPipeline pipeline;
    public static boolean initialized = false;
    public static boolean streaming = false;

    public static void initializeCV(HardwareManager manager)
    {
        int cameraMonitorViewId = manager.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", manager.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(manager.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CVPipeline();
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                initialized = true;
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public static void startStream()
    {

        if (!initialized)
        {
            long sysTime = System.currentTimeMillis() + 2500;

            while (!initialized && System.currentTimeMillis() < sysTime)
            {
            }

            if (!initialized)
                return; //If still not initialized then cancel operation.
        }

        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);


    }

    public static void stopStream()
    {
        webcam.stopStreaming();
        streaming = true;
    }

    public static Mat grabFrame()
    {
        if (streaming)
            return CVPipeline.lastFrame;
        return null;
    }

    public static class CVPipeline extends OpenCvPipeline {

        static boolean viewportPaused;
        public static Mat lastFrame;

        @Override
        public Mat processFrame(Mat input) {
            streaming = true;
            lastFrame = input;
            return null;
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}
