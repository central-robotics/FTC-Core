package com.chsrobotics.ftccore.teleop;

import android.icu.number.Scale;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.engine.navigation.control.PID;
import com.chsrobotics.ftccore.engine.navigation.control.PIDParams;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.Vector;
import java.util.concurrent.TimeUnit;

public class Drive {
    public final HardwareManager manager;
    public static double linearMax = 1;
    public static double rotationMax = 1;
    public boolean overrideDriveControl = false;
    private boolean isCounterClockwise = false;
    private boolean correctPID = false;
    private PIDCoefficients correctPIDCoeffs;
    private long prevTime = System.currentTimeMillis();
    private final UserDriveLoop loop;
    private final Builder.ScaleMode mode;
    private double currentAngle;

    public Drive (HardwareManager manager, UserDriveLoop loop, Builder.ScaleMode mode, PIDCoefficients correctPID)
    {
        this.correctPIDCoeffs = correctPID;
        this.mode = mode;
        this.manager = manager;
        this.loop = loop;
        loop.drive = this;
    }

    private void driveLoop()
    {
        correctPID = correctPIDCoeffs != null;

        while (!manager.opMode.isStopRequested())
        {
            loop.loop();

            prevTime = System.currentTimeMillis();

            Gamepad gamepad1 = manager.opMode.gamepad1;

            double joystick_y;
            double joystick_x;
            double joystick_power = 0;
            double negative_power;
            double positive_power;
            double theta;
            double orientation;
            double rot_power;
            Orientation gyro_angles;

            joystick_y = -gamepad1.left_stick_y;
            joystick_x = gamepad1.left_stick_x == 0 ? 0.001 : gamepad1.left_stick_x;

            rot_power = (manager.thetaReversed ? -1 : 1) * gamepad1.right_stick_x;

            double rawPower = Math.sqrt(Math.pow(joystick_x, 2) + Math.pow(joystick_y, 2));

            switch (mode)
            {
                case SIN_SCALE:
                    joystick_power = (Math.sin((rawPower * (Math.PI/2)) - (Math.PI / 2d))) + 1;
                    break;
                case QUADRATIC_SCALE:
                    joystick_power = Math.pow(rawPower, 2);
                    break;
                default:
                    joystick_power = rawPower;
                    break;
            }

            gyro_angles = manager.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            theta = gyro_angles.firstAngle - manager.IMUReset - manager.offset;

            theta *= manager.thetaReversed ? -1 : 1;

            orientation = (Math.atan2(joystick_y, joystick_x) - Math.PI / 4) - theta;

            negative_power = (joystick_power * Math.sin(orientation));
            positive_power = (orientation != 0) ? (joystick_power * Math.cos(orientation)) :
                    negative_power;

            if (!overrideDriveControl)
                move(positive_power, negative_power, rot_power, false);
        }
    }

    public void move(double posinput, double neginput, double rotinput, boolean usePID)
    {
        manager.getLeftFrontMotor().setVelocity((manager.linearMax * -posinput) - (manager.rotMax * rotinput));
        manager.getRightFrontMotor().setVelocity((manager.linearMax * neginput) - (manager.rotMax * rotinput));
        manager.getLeftBackMotor().setVelocity((manager.linearMax * -neginput) - (manager.rotMax * rotinput));
        manager.getRightBackMotor().setVelocity((manager.linearMax * posinput) - (manager.rotMax * rotinput));
    }

    public void runDriveLoop() {
        driveLoop();
    }

    public static class Builder
    {
        private final HardwareManager manager;
        private UserDriveLoop loop;
        private ScaleMode mode = ScaleMode.LINEAR;
        private PIDCoefficients correctPID = null;

        public Builder(HardwareManager manager)
        {
            this.manager = manager;
        }

        public Builder addUserLoop(UserDriveLoop loop)
        {
            this.loop = loop;
            return this;
        }

        public Builder useScale(ScaleMode mode)
        {
            this.mode = mode;
            return this;
        }

        public Builder enableTurnCorrection(PIDCoefficients correctPID)
        {
            this.correctPID = correctPID;
            return this;
        }

        public enum ScaleMode
        {
            LINEAR,
            SIN_SCALE,
            QUADRATIC_SCALE
        }

        public Drive build()
        {
            return new Drive(manager, loop, mode, correctPID);
        }
    }
}
