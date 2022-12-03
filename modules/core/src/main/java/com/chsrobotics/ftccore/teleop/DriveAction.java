package com.chsrobotics.ftccore.teleop;

import com.chsrobotics.ftccore.actions.Action;

public class DriveAction {
    public final Action action;
    public boolean trigger;

    public DriveAction(boolean trigger, Action action)
    {
        this.action = action;
        this.trigger = trigger;
    }
}
