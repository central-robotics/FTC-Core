package com.chsrobotics.ftccore.engine.localization;


import com.chsrobotics.ftccore.engine.localization.localizer.EncoderLocalizer;
import com.chsrobotics.ftccore.engine.localization.localizer.IMULocalizer;
import com.chsrobotics.ftccore.engine.localization.localizer.Localizer;
import com.chsrobotics.ftccore.engine.localization.localizer.OdometryLocalizer;
import com.chsrobotics.ftccore.engine.localization.localizer.VisionLocalizer;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.ArrayList;
import java.util.List;

public class LocalizationEngine {
    /*
    The localization engine can be used by actions or anywhere else across the system. It is responsible for fusing the
    inputs of the different localizers using a Kalman filter into a more accurate output.
    */

    public static Position position;

    public Position currentPosition = new Position(0, 0, 0);
    public Position lastPosition = new Position(0, 0, 0);

    private List<Localizer> localizers;

    private final HardwareManager hardware;

    public LocalizationEngine(HardwareManager hardware)
    {
        this.hardware = hardware;
        initializeLocalization();
    }

    /**
     * Computes position of robot through a Kalman filter.
     * @return Current position of the robot in X, Y, and T (radians)
     */
    public Position getCurrentPosition()
    {
        //Kalman filter
        List<Position> positions = new ArrayList<>();

//        for (Localizer localizer : localizers) {
//            positions.add(localizer.getRobotPosition(lastPosition));
//        } STUTTERING ISSUE

        lastPosition = currentPosition;
        currentPosition = localizers.get(0).getRobotPosition(lastPosition);
        position = currentPosition;
        return currentPosition; //Temporarily returning only encoder based position.
    }

    private void initializeLocalization()
    {
        localizers = new ArrayList<>();

        if (hardware.accessoryOdometryPods.length == 2) {
            localizers.add(new OdometryLocalizer(null, hardware));
        }

        if (hardware.driveMotors.length == 4) {
            localizers.add(new EncoderLocalizer(null, hardware));
        }

        if (hardware.isImuLocalEnabled()) {
            localizers.add(new IMULocalizer(null, hardware));
        }

        if (hardware.accessoryCameras.length > 0) {
            localizers.add(new VisionLocalizer(null, hardware));
        }
    }
}
