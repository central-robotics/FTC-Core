package com.chsrobotics.ftccore.teleop;

import com.chsrobotics.ftccore.actions.Action;

public class DriveAction {
    public final Action action;
    public final Drive.Builder.GamepadButtons bindedButton;
    public final boolean actuation;
    public final int gamepad;

    public DriveAction(Action action, Drive.Builder.GamepadButtons button , int gamepad, boolean actuation)
    {
        this.action = action;
        this.bindedButton = button;
        this.gamepad = gamepad;
        this.actuation = actuation;
    }
}
