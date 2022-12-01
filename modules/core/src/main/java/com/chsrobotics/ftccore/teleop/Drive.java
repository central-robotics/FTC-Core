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

            performDriveAction(manager.opMode.gamepad1);
            performDriveAction(manager.opMode.gamepad2);

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

            joystick_y = gamepad1.left_stick_y > 0 ? Math.pow(gamepad1.left_stick_y, 2) :
                    -Math.pow(gamepad1.left_stick_y, 2);
            joystick_x = (gamepad1.left_stick_x == 0) ? 0.000001 :
                    (gamepad1.left_stick_x > 0 ? Math.pow(gamepad1.left_stick_x, 2) :
                            -Math.pow(gamepad1.left_stick_x, 2));

            rot_power = (gamepad1.right_stick_x);

            joystick_power = Math.sqrt(Math.pow(joystick_x, 2) + Math.pow(joystick_y, 2));

            gyro_angles = manager.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            theta = gyro_angles.firstAngle - manager.IMUReset;

            orientation = (joystick_x > 0) ? (Math.atan(-joystick_y / joystick_x) - Math.PI / 4) - theta :
                    (Math.atan(-joystick_y / joystick_x) + Math.PI - Math.PI / 4) - theta;

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
        manager.getLeftBackMotor().setPower((manager.linearSpeed * -neginput)- (manager.rotSpeed *rotinput));
        manager.getRightBackMotor().setPower((manager.linearSpeed * posinput)- (manager.rotSpeed *rotinput));
    }

    private void performDriveAction(Gamepad gamepad)
    {
        int gp = gamepad == manager.opMode.gamepad1 ? 1 : 2;
        for (DriveAction action : actions)
        {
            if (gamepad.a && action.bindedButton == Builder.GamepadButtons.A && action.gamepad == gp)
                action.action.execute();
            if (gamepad.b && action.bindedButton == Builder.GamepadButtons.B && action.gamepad == gp)
                action.action.execute();
            if (gamepad.y && action.bindedButton == Builder.GamepadButtons.Y && action.gamepad == gp)
                action.action.execute();
            if (gamepad.x && action.bindedButton == Builder.GamepadButtons.X && action.gamepad == gp)
                action.action.execute();
            if (gamepad.triangle && action.bindedButton == Builder.GamepadButtons.Triangle && action.gamepad == gp)
                action.action.execute();
            if (gamepad.cross && action.bindedButton == Builder.GamepadButtons.Cross && action.gamepad == gp)
                action.action.execute();
            if (gamepad.circle && action.bindedButton == Builder.GamepadButtons.Circle && action.gamepad == gp)
                action.action.execute();
            if (gamepad.dpad_left && action.bindedButton == Builder.GamepadButtons.DPLEFT && action.gamepad == gp)
                action.action.execute();
            if (gamepad.dpad_down && action.bindedButton == Builder.GamepadButtons.DPDOWN && action.gamepad == gp)
                action.action.execute();
            if (gamepad.dpad_right && action.bindedButton == Builder.GamepadButtons.DPRIGHT && action.gamepad == gp)
                action.action.execute();
            if (gamepad.dpad_up && action.bindedButton == Builder.GamepadButtons.DPUP && action.gamepad == gp)
                action.action.execute();
            if (gamepad.right_bumper && action.bindedButton == Builder.GamepadButtons.RB && action.gamepad == gp)
                action.action.execute();
            if (gamepad.left_bumper && action.bindedButton == Builder.GamepadButtons.LB && action.gamepad == gp)
                action.action.execute();
            if (gamepad.right_trigger > 0.001 && action.bindedButton == Builder.GamepadButtons.RT && action.gamepad == gp)
                action.action.execute();
            if (gamepad.left_trigger > 0.001 && action.bindedButton == Builder.GamepadButtons.LT && action.gamepad == gp)
                action.action.execute();
            if (!gamepad.a && action.bindedButton == Builder.GamepadButtons.A && action.gamepad == gp && !action.actuation)
                action.action.execute();
            if (!gamepad.b && action.bindedButton == Builder.GamepadButtons.B && action.gamepad == gp && !action.actuation)
                action.action.execute();
            if (!gamepad.y && action.bindedButton == Builder.GamepadButtons.Y && action.gamepad == gp && !action.actuation)
                action.action.execute();
            if (!gamepad.x && action.bindedButton == Builder.GamepadButtons.X && action.gamepad == gp && !action.actuation)
                action.action.execute();
            if (!gamepad.triangle && action.bindedButton == Builder.GamepadButtons.Triangle && action.gamepad == gp && !action.actuation)
                action.action.execute();
            if (!gamepad.cross && action.bindedButton == Builder.GamepadButtons.Cross && action.gamepad == gp && !action.actuation)
                action.action.execute();
            if (!gamepad.circle && action.bindedButton == Builder.GamepadButtons.Circle && action.gamepad == gp && !action.actuation)
                action.action.execute();
            if (!gamepad.dpad_left && action.bindedButton == Builder.GamepadButtons.DPLEFT && action.gamepad == gp && !action.actuation)
                action.action.execute();
            if (!gamepad.dpad_down && action.bindedButton == Builder.GamepadButtons.DPDOWN && action.gamepad == gp && !action.actuation)
                action.action.execute();
            if (!gamepad.dpad_right && action.bindedButton == Builder.GamepadButtons.DPRIGHT && action.gamepad == gp && !action.actuation)
                action.action.execute();
            if (!gamepad.dpad_up && action.bindedButton == Builder.GamepadButtons.DPUP && action.gamepad == gp && !action.actuation)
                action.action.execute();
            if (!gamepad.right_bumper && action.bindedButton == Builder.GamepadButtons.RB && action.gamepad == gp && !action.actuation)
                action.action.execute();
            if (!gamepad.left_bumper && action.bindedButton == Builder.GamepadButtons.LB && action.gamepad == gp && !action.actuation)
                action.action.execute();
            if (!(gamepad.right_trigger > 0.001) && action.bindedButton == Builder.GamepadButtons.RT && action.gamepad == gp && !action.actuation)
                action.action.execute();
            if (!(gamepad.left_trigger > 0.001) && action.bindedButton == Builder.GamepadButtons.LT && action.gamepad == gp && !action.actuation)
                action.action.execute();
        }
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

        public Builder bindActionToButton(GamepadButtons button, Action action, int gamepad, boolean actuation)
        {
            actions.add(new DriveAction(action, button, gamepad, actuation));
            return this;
        }

        public Builder addUserLoop(UserDriveLoop loop)
        {
            this.loop = loop;
            return this;
        }

        public Drive Build()
        {
            return new Drive(manager, actions, loop);
        }

        public enum GamepadButtons
        {
            Y,
            B,
            A,
            X,
            Square,
            Triangle,
            Cross,
            Circle,
            RB,
            LB,
            DPRIGHT,
            DPDOWN,
            DPLEFT,
            DPUP,
            RT,
            LT
        }
    }
}
