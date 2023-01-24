package com.chsrobotics.ftccore.engine.navigation.path;

import com.chsrobotics.ftccore.geometry.Position;

public class TrapezoidalMotionProfileWithRot extends MotionProfile {
    private double angleDist = 0;
    private double maxRotAccel = 0;
    private double maxRotVelocity = 0;

    public TrapezoidalMotionProfileWithRot(double accel, double velocity, double rotAccel, double rotVelocity) {
        super(accel, velocity);
        maxRotAccel = rotAccel;
        maxRotVelocity = rotVelocity;
    }

    public void calculateProfile(Position p0, Position p1, double currentAngle, double targetAngle) {
        super.calculateProfile(p0, p1);
        angleDist = targetAngle - currentAngle;
    }

    public double getAngleOutput(double time)
    {
        double accelerationTime = maxRotVelocity / maxRotAccel;
        double halfwayDistance = angleDist / 2;
        double accelerationDistance = 0.5 * maxRotAccel * (accelerationTime * accelerationTime);

        if (accelerationDistance > halfwayDistance)
            accelerationTime = Math.sqrt(halfwayDistance / (0.5 * maxRotAccel));

        accelerationDistance = 0.5 * maxRotAccel * (accelerationTime * accelerationTime);

        maxRotVelocity = maxRotAccel * accelerationTime;

        double deaccelTime = accelerationTime;

        double cruiseDistance = angleDist - 2 * accelerationDistance;
        double cruiseTime = cruiseDistance / maxRotVelocity;
        deaccelTime = accelerationTime + cruiseTime;

        double calculatedTime = accelerationTime + cruiseTime + deaccelTime;

        if (time > calculatedTime)
            return distance;

        if (time < accelerationTime)
            return 0.5 * maxRotAccel * (time * time);
        else if (time < deaccelTime) {
            accelerationDistance = 0.5 * maxRotAccel * (accelerationTime * accelerationTime);
            double cruiseCurrentTime = time - accelerationTime;

            return accelerationDistance + maxRotVelocity * cruiseCurrentTime;
        } else {
            accelerationDistance = 0.5 * maxRotAccel * (accelerationTime * accelerationTime);
            cruiseDistance = maxRotVelocity * cruiseTime;
            deaccelTime = time - deaccelTime;

            // use the kinematic equations to calculate the instantaneous desired position
            return accelerationDistance + cruiseDistance + maxRotVelocity * deaccelTime - 0.5 * maxRotAccel * (deaccelTime * deaccelTime);
        }
    }

    @Override
    public double getOutput(double time) {
        double accelerationTime = maxVelocity / maxAccel;
        double halfwayDistance = distance / 2;
        double accelerationDistance = 0.5 * maxAccel * (accelerationTime * accelerationTime);

        if (accelerationDistance > halfwayDistance)
            accelerationTime = Math.sqrt(halfwayDistance / (0.5 * maxAccel));

        accelerationDistance = 0.5 * maxAccel * (accelerationTime * accelerationTime);

        maxVelocity = maxAccel * accelerationTime;

        double deaccelTime = accelerationTime;

        double cruiseDistance = distance - 2 * accelerationDistance;
        double cruiseTime = cruiseDistance / maxVelocity;
        deaccelTime = accelerationTime + cruiseTime;

        double calculatedTime = accelerationTime + cruiseTime + deaccelTime;

        if (time > calculatedTime)
            return distance;

        if (time < accelerationTime)
            return 0.5 * maxAccel * (time * time);
        else if (time < deaccelTime) {
            accelerationDistance = 0.5 * maxAccel * (accelerationTime * accelerationTime);
            double cruiseCurrentTime = time - accelerationTime;

            return accelerationDistance + maxVelocity * cruiseCurrentTime;
        } else {
            accelerationDistance = 0.5 * maxAccel * (accelerationTime * accelerationTime);
            cruiseDistance = maxVelocity * cruiseTime;
            deaccelTime = time - deaccelTime;

            // use the kinematic equations to calculate the instantaneous desired position
            return accelerationDistance + cruiseDistance + maxVelocity * deaccelTime - 0.5 * maxAccel * (deaccelTime * deaccelTime);
        }

    }
}
