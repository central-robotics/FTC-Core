package com.chsrobotics.ftccore.engine.localization;


import com.chsrobotics.ftccore.engine.localization.localizer.EncoderLocalizer;
import com.chsrobotics.ftccore.engine.localization.localizer.IMULocalizer;
import com.chsrobotics.ftccore.engine.localization.localizer.Localizer;
import com.chsrobotics.ftccore.engine.localization.localizer.VisionLocalizer;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class LocalizationEngine {
    private Position currentPosition;
    private Position lastPosition;

    private Localizer[] localizers;

    private final HardwareManager hardware;

    public LocalizationEngine(HardwareManager hardware)
    {
        this.hardware = hardware;

        initializeLocalization();
    }

    public Position getCurrentPosition()
    {
        Position[] positions = new Position[localizers.length];

        for (int i = 0; i < localizers.length; i++)
        {
            positions[i] = localizers[i].getRobotPosition();
        }

        localizers[0].updateRobotPosition(positions[0]);

        return positions[0]; //Temporarily returning only encoder based position.
    }

    private void initializeLocalization()
    {
        int localizerCapacity = 0;

        if (hardware.driveMotors.length == 4)
            localizerCapacity++;

        if (hardware.imuLocalEnabled)
            localizerCapacity++;

        if (hardware.accessoryCameras.length > 0)
            localizerCapacity++;

        localizers = new Localizer[localizerCapacity];

        if (localizerCapacity > 0)
        {
            if (hardware.driveMotors.length == 4)
                localizers[0] = new EncoderLocalizer(null, hardware);

            if (hardware.imuLocalEnabled)
                localizers[1] = new IMULocalizer(null, hardware);

            if (hardware.accessoryCameras.length > 0)
                localizers[2] = new VisionLocalizer(null, hardware);
        }
    }
}
