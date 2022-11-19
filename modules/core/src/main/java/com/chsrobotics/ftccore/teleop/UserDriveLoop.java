package com.chsrobotics.ftccore.teleop;

import com.chsrobotics.ftccore.hardware.HardwareManager;

public abstract class UserDriveLoop {

    protected HardwareManager hardware;
    public UserDriveLoop(HardwareManager manager)
    {
        hardware = manager;
    }
    public abstract void loop();
}
