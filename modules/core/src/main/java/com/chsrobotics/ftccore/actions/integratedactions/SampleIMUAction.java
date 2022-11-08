package com.chsrobotics.ftccore.actions.integratedactions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class SampleIMUAction extends Action {
    public SampleIMUAction(HardwareManager hardware) {
        super(hardware);
    }

    @Override
    public void execute() {
        hardware.IMUReset = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }
}
