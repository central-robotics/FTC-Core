package com.chsrobotics.ftccore.engine.localization.localizer;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.management.constants.MiscConstants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class EncoderLocalizer extends Localizer{

    /*
    Encoder localization is different from DeadWheelLocalizer. Instead of using odometry pods, encoder localization utilizes
    the motor encoders on the four drive motors that are used to drive the 4 wheels on the robot. While this is slightly less
    accurate than pods, this method is still good for teams that do not have pods.
    */

    HardwareManager hardware;

    private int lastLfPos, lastRfPos, lastRbPos, lastLbPos;
    private double mmPerTick;

    public EncoderLocalizer(Position initialState, HardwareManager hardware) {
        super(initialState, hardware);
        this.hardware = hardware;
        mmPerTick = (Math.PI * hardware.wheelDiameter) / hardware.encoderRes;
    }

    @Override
    public Position getRobotPosition(Position previousPosition) {
        //Encoder values. These are in ticks. We will later convert this to a usable distance.
        int lfPos, rfPos, rbPos, lbPos;

        //Record encoder values.
        lfPos = hardware.getLeftFrontMotor().getCurrentPosition();
        rfPos = hardware.getRightFrontMotor().getCurrentPosition();
        rbPos = hardware.getRightBackMotor().getCurrentPosition();
        lbPos = hardware.getLeftBackMotor().getCurrentPosition();

        //Displacement values
        double lfDisp, rfDisp, rbDisp, lbDisp;

        //Calculate displacement values
        lfDisp = (lfPos - lastLfPos) * mmPerTick;
        rfDisp = (rfPos - lastRfPos) * mmPerTick;
        rbDisp = (rbPos - lastRbPos) * mmPerTick;
        lbDisp = (lbPos - lastLbPos) * mmPerTick;

        //Store encoder values so we can use them in calculating displacement.
        lastLfPos = lfPos;
        lastRfPos = rfPos;
        lastRbPos = rbPos;
        lastLbPos = lbPos;

        //Average of displacement values
        double dispAvg;

        //Calculate avg.
        dispAvg = (lfDisp + rfDisp + rbDisp + lbDisp) / 4;

        //Robot displacement
        double robotLfDisp, robotRfDisp, robotRbDisp, robotLbDisp;

        //Calculate robot displacement
        robotLfDisp = lfDisp - dispAvg;
        robotRfDisp = rfDisp - dispAvg;
        robotRbDisp = rbDisp - dispAvg;
        robotLbDisp = lbDisp - dispAvg;

        //Holonomic displacement in robot reference frame.
        double deltaX, deltaY;

        //Compute displacement in robot reference frame.
        deltaX = (robotLfDisp + robotRfDisp - robotRbDisp - robotLbDisp) / (2 * Math.sqrt(2));
        deltaY = (robotLfDisp - robotRfDisp - robotRbDisp + robotLbDisp) / (2 * Math.sqrt(2));

        //Robot theta
        double theta;

        //Compute robot theta
        theta = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - hardware.offset;
        if (theta > 2 * Math.PI) {
            theta -= 2 * Math.PI;
        } else if (theta < 0) {
            theta += 2 * Math.PI;
        }

        //Compute displacement in field reference frame.
        double deltaXf = deltaX * Math.cos(theta) - deltaY * Math.sin(theta);
        double deltaYf = deltaY * Math.cos(theta) + deltaX * Math.sin(theta);

        Position robotPosition = new Position();
        robotPosition.x = previousPosition.x - deltaXf;
        robotPosition.y = previousPosition.y - deltaYf;
        robotPosition.t = theta;

        if (robotPosition.x == 0) robotPosition.x = 0.0000001;

        return robotPosition;
    }

    @Override
    public void updateRobotPosition(Position pos) {

    }
}
