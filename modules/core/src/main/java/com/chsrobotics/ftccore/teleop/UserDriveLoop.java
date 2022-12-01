package com.chsrobotics.ftccore.teleop;

import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class UserDriveLoop {

    protected HardwareManager hardware;
    protected OpMode opmode;
    public UserDriveLoop(HardwareManager manager, OpMode mode)
    {
        opmode = mode;
        hardware = manager;
    }
    public abstract void loop();
}
