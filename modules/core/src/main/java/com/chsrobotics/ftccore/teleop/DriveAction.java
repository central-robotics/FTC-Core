package com.chsrobotics.ftccore.teleop;

import com.chsrobotics.ftccore.actions.Action;

public class DriveAction {
    public final Action action;
    public final Drive.Builder.GamepadButtons bindedButton;
    public final int gamepad;

    public DriveAction(Action action, Drive.Builder.GamepadButtons button, int gamepad)
    {
        this.action = action;
        this.bindedButton = button;
        this.gamepad = gamepad;
    }
}
