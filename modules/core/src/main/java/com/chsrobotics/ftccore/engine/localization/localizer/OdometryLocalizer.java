package com.chsrobotics.ftccore.engine.localization.localizer;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.management.constants.MiscConstants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class OdometryLocalizer extends Localizer{

    /*
    Encoder localization is different from DeadWheelLocalizer. Instead of using odometry pods, encoder localization utilizes
    the motor encoders on the four drive motors that are used to drive the 4 wheels on the robot. While this is slightly less
    accurate than pods, this method is still good for teams that do not have pods.
    */

    HardwareManager hardware;

    private int lastLat = 0, lastLon = 0;
    private double lastHeading = 0;

    private final double mmPerTick;

    public OdometryLocalizer(Position initialState, HardwareManager hardware) {
        super(initialState, hardware);
        this.hardware = hardware;
        mmPerTick = (hardware.wheelDiameter * Math.PI) / hardware.encoderRes;
    }

//    public double getThetaDifference(double current, double old) {
//        double headingError = current - old;
//        boolean isCounterClockwise = false;
//
//        if (Math.abs(current - (old - (2 * Math.PI))) < Math.abs(headingError))
//        {
//            headingError = current - (old - (2 * Math.PI));
//            isCounterClockwise = true;
//        }
//
//        if (Math.abs(current - (old + (2 * Math.PI))) < headingError)
//        {
//            headingError = current - (old + (2 * Math.PI));
//            isCounterClockwise = true;
//        }
//
//        if (headingError > 0 && (headingError < Math.PI)) {
//            isCounterClockwise = true;
//        }
//
//        if (headingError < 0 && (headingError > -Math.PI)) {
//            isCounterClockwise = false;
//        }
//
//        return Math.abs(headingError) * (isCounterClockwise ? 1 : -1);
//    }

    @Override
    public Position getRobotPosition(Position previousPosition) {
        //Encoder values. These are in ticks. We will later convert this to a usable distance.

        //Record encoder values.
        int lat = hardware.accessoryOdometryPods[0].getCurrentPosition();
        int lon = hardware.accessoryOdometryPods[1].getCurrentPosition();

        double heading = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - hardware.offset;
        double d_heading = heading - lastHeading;

        //Calculate displacement values in mm
        //Account for active rotation by subtracting the arc length of heading displacement
        //TODO: Make distances from rotation origin configurable
        double d_lat = ((lat - lastLat) * mmPerTick) - (hardware.latWheelOffset * d_heading);
        double d_lon = ((lon - lastLon) * mmPerTick) - (hardware.lonWheelOffset * d_heading);

        //Account for current orientation
        double dx = d_lat * Math.cos(heading) + d_lon * Math.sin(heading);
        double dy = d_lon * Math.cos(heading) - d_lat * Math.sin(heading);

        //Store encoder values so we can use them in calculating displacement.
        lastLat = lat;
        lastLon = lon;
        lastHeading = heading;

        //Update robot position
        Position robotPosition = new Position();
        robotPosition.x = previousPosition.x - dx;
        robotPosition.y = previousPosition.y + dy;
        robotPosition.t = heading;

        if (robotPosition.x == 0) robotPosition.x = 0.0000001;

        return robotPosition;
    }

    @Override
    public void updateRobotPosition(Position pos) {

    }
}
