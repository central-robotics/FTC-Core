package com.chsrobotics.ftccore.engine.localization;


import com.chsrobotics.ftccore.engine.localization.localizer.EncoderLocalizer;
import com.chsrobotics.ftccore.engine.localization.localizer.IMULocalizer;
import com.chsrobotics.ftccore.engine.localization.localizer.Localizer;
import com.chsrobotics.ftccore.engine.localization.localizer.VisionLocalizer;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;

import java.util.ArrayList;
import java.util.List;

public class LocalizationEngine {
    private Position currentPosition;
    private Position lastPosition;

    private List<Localizer> localizers;

    private final HardwareManager hardware;

    public LocalizationEngine(HardwareManager hardware)
    {
        this.hardware = hardware;

        initializeLocalization();
    }

    public Position getCurrentPosition()
    {
        List<Position> positions = new ArrayList<>();

        for (Localizer localizer : localizers) {
            positions.add(localizer.getRobotPosition());
        }

        localizers.get(0).updateRobotPosition(positions.get(0));

        return positions.get(0); //Temporarily returning only encoder based position.
    }

    private void initializeLocalization()
    {
        localizers = new ArrayList<>();

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
