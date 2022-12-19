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
    private double lastLatAdjusted = 0, lastLonAdjusted = 0;
    private double lastHeading;

    public OdometryLocalizer(Position initialState, HardwareManager hardware) {
        super(initialState, hardware);
        this.hardware = hardware;
    }


    public double getThetaDifference(double current, double old) {
        double headingError = current - old;
        boolean isCounterClockwise = false;

        if (Math.abs(current - (old - (2 * Math.PI))) < Math.abs(headingError))
        {
            headingError = current - (old - (2 * Math.PI));
            isCounterClockwise = true;
        }

        if (Math.abs(current - (old + (2 * Math.PI))) < headingError)
        {
            headingError = current - (old + (2 * Math.PI));
            isCounterClockwise = true;
        }

        if (headingError > 0 && (headingError < Math.PI)) {
            isCounterClockwise = true;
        }

        if (headingError < 0 && (headingError > -Math.PI)) {
            isCounterClockwise = false;
        }

        return Math.abs(headingError) * (isCounterClockwise ? 1 : -1);
    }

    @Override
    public Position getRobotPosition(Position previousPosition) {
        //Encoder values. These are in ticks. We will later convert this to a usable distance.

        //Record encoder values.
        int lat = hardware.accessoryOdometryPods[0].getCurrentPosition();
        int lon = hardware.accessoryOdometryPods[1].getCurrentPosition();

        double heading = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - hardware.offset;
        if (heading > 2 * Math.PI) {
            heading -= 2 * Math.PI;
        } else if (heading < 0) {
            heading += 2 * Math.PI;
        }

        //Calculate displacement values in mm
        double deltaX = (lat - lastLat) * 0.01358149;
        double deltaY = (lon - lastLon) * 0.01358149;

        double magnitude = Math.sqrt((deltaY * deltaY) + (deltaX * deltaX));
        double direction = -Math.atan2(deltaX, deltaY) - heading;

        double latAdjusted = Math.sin(direction) * magnitude;
        double lonAdjusted = Math.cos(direction) * magnitude;

//        double deltaXf = latAdjusted - lastLatAdjusted;
//        double deltaYf = lonAdjusted - lastLonAdjusted;

        //Store encoder values so we can use them in calculating displacement.
        lastLat = lat;
        lastLon = lon;
        lastLatAdjusted = latAdjusted;
        lastLonAdjusted = lonAdjusted;

        Position robotPosition = new Position();
        robotPosition.x = previousPosition.x + latAdjusted;
        robotPosition.y = previousPosition.y + lonAdjusted;
        robotPosition.t = heading;

        if (robotPosition.x == 0) robotPosition.x = 0.0000001;

        if (false) {
            hardware.opMode.telemetry.addData("deltaX", deltaX);
            hardware.opMode.telemetry.addData("deltaY", deltaY);
            hardware.opMode.telemetry.addData("direction", direction);
            hardware.opMode.telemetry.addData("heading", heading);
            hardware.opMode.telemetry.addData("magnitude", magnitude);
            hardware.opMode.telemetry.addData("latAdjusted", latAdjusted);
            hardware.opMode.telemetry.addData("lonAdjusted", lonAdjusted);
//        hardware.opMode.telemetry.addData("deltaXf", deltaXf);
//        hardware.opMode.telemetry.addData("deltaYf", deltaYf);
            hardware.opMode.telemetry.addData("robotX", robotPosition.x);
            hardware.opMode.telemetry.addData("robotY", robotPosition.y);
            hardware.opMode.telemetry.addData("rawX", hardware.accessoryOdometryPods[0].getCurrentPosition());
            hardware.opMode.telemetry.addData("rawY", hardware.accessoryOdometryPods[1].getCurrentPosition());
        }

        return robotPosition;

        //Calculate net displacement after accounting for rotation

        //Holonomic displacement in robot reference frame.
//        double deltaX, deltaY;

        //Compute displacement in robot reference frame.
//        deltaX = (robotLfDisp + robotRfDisp - robotRbDisp - robotLbDisp) / (2 * Math.sqrt(2));
//        deltaY = (robotLfDisp - robotRfDisp - robotRbDisp + robotLbDisp) / (2 * Math.sqrt(2));

        //Robot heading
/*        double heading;

        //Compute robot heading
        heading = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - hardware.offset;
        if (heading > 2 * Math.PI) {
            heading -= 2 * Math.PI;
        } else if (heading < 0) {
            heading += 2 * Math.PI;
        }

        //Compute displacement in field reference frame.
        double deltaXf = deltaX * Math.cos(heading) - deltaY * Math.sin(heading);
        double deltaYf = deltaY * Math.cos(heading) + deltaX * Math.sin(heading);

        Position robotPosition = new Position();
        robotPosition.x = previousPosition.x - deltaXf;
        robotPosition.y = previousPosition.y - deltaYf;
        robotPosition.t = heading;

        if (robotPosition.x == 0) robotPosition.x = 0.0000001;

        return robotPosition;*/
    }

    @Override
    public void updateRobotPosition(Position pos) {

    }
}
