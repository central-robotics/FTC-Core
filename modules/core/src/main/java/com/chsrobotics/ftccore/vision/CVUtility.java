package com.chsrobotics.ftccore.vision;

import com.chsrobotics.ftccore.hardware.HardwareManager;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class CVUtility {

    private OpenCvCamera camera;
    private InterfacePipeline pipeline;
    public boolean initialized = false;

    public CVUtility(HardwareManager manager)
    {
        if (!manager.useCV)
            return;

        camera = OpenCvCameraFactory.getInstance().createWebcam(manager.accessoryCameras[0]);

        camera.setPipeline(pipeline = new InterfacePipeline());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                initialized = true;
            }

            @Override
            public void onError(int errorCode) {
                manager.useCV = false;
            }
        });
    }



    public void startStreaming()
    {
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public Mat grabFrame()
    {
        return pipeline.latestFrame;
    }

    public void stopStreaming()
    {
        camera.stopStreaming();
    }

    public class InterfacePipeline extends OpenCvPipeline
    {
        public Mat latestFrame;

        @Override
        public Mat processFrame(Mat input) {
            latestFrame = input;
            return null;
        }
    }
}
