package com.chsrobotics.ftccore.teleop;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.Vector;

public class Drive {
    private final HardwareManager manager;
    private final ArrayList<DriveAction> actions;
    private long prevTime = System.currentTimeMillis();
    private final UserDriveLoop loop;

    public Drive (HardwareManager manager, ArrayList<DriveAction> actions, UserDriveLoop loop)
    {
        this.manager = manager;
        this.actions = actions;
        this.loop = loop;
    }

    private void driveLoop()
    {
        while (!manager.opMode.isStopRequested())
        {
            loop.loop();

            prevTime = System.currentTimeMillis();

            for (DriveAction action : actions)
            {
                if (action.trigger)
                    action.action.execute();
            }

            Gamepad gamepad1 = manager.opMode.gamepad1;

            double joystick_y;
            double joystick_x;
            double joystick_power;
            double negative_power;
            double positive_power;
            double theta;
            double orientation;
            double rot_power;
            Orientation gyro_angles;

            joystick_y = -gamepad1.left_stick_y;
            joystick_x = (gamepad1.left_stick_x == 0) ? 0.000001 : -gamepad1.left_stick_x;

            rot_power = (gamepad1.right_stick_x);

            joystick_power = Math.sqrt(Math.pow(joystick_x, 2) + Math.pow(joystick_y, 2));

            gyro_angles = manager.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            theta = gyro_angles.firstAngle - manager.IMUReset - manager.offset;

            orientation = (joystick_x > 0) ? (Math.atan(joystick_y / joystick_x) - Math.PI / 4) + theta :
                    (Math.atan(joystick_y / joystick_x) + Math.PI - Math.PI / 4) + theta;

            negative_power = (joystick_power * Math.sin(orientation));
            positive_power = (orientation != 0) ? (joystick_power * Math.cos(orientation)) :
                    negative_power;

            move(positive_power, negative_power, rot_power);
        }
    }

    public void move(double posinput, double neginput, double rotinput)
    {
        manager.getLeftFrontMotor().setPower((manager.linearSpeed * -posinput) - (manager.rotSpeed *rotinput));
        manager.getRightFrontMotor().setPower((manager.linearSpeed * neginput) - (manager.rotSpeed *rotinput));
        manager.getLeftBackMotor().setPower((manager.linearSpeed * -neginput) - (manager.rotSpeed *rotinput));
        manager.getRightBackMotor().setPower((manager.linearSpeed * posinput) - (manager.rotSpeed *rotinput));
    }

    public void runDriveLoop() {
        driveLoop();
    }

    public static class Builder
    {
        private final HardwareManager manager;
        private final ArrayList<DriveAction> actions;
        private UserDriveLoop loop;

        public Builder(HardwareManager manager)
        {
            this.actions = new ArrayList<>();
            this.manager = manager;
        }

        public Builder bindActionToButton(boolean trigger, Action action)
        {
            actions.add(new DriveAction(trigger, action));
            return this;
        }

        public Builder addUserLoop(UserDriveLoop loop)
        {
            this.loop = loop;
            return this;
        }

        public Drive build()
        {
            return new Drive(manager, actions, loop);
        }
    }
}
