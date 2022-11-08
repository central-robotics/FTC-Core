package com.chsrobotics.ftccore.actions.integratedactions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class SampleAccessoryMotor extends Action {

    /*
    This is an example of a basic action.
    The 0th accessory motor spins forward at 20% power until it has reached a tick count of 1000.
    This implementation is standard across all basic actions.
     */

    public SampleAccessoryMotor(HardwareManager hardware) {
        super(hardware);
    }

    @Override
    public void execute() {
        DcMotorEx motor = hardware.accessoryMotors[0];

        while (motor.getCurrentPosition() < 1000)
            motor.setPower(0.2);
        motor.setPower(0);
    }
}
