package com.chsrobotics.ftccore.engine.navigation;

import com.chsrobotics.ftccore.engine.localization.LocalizationEngine;
import com.chsrobotics.ftccore.geometry.Position;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MotionProfileExample {

    private LocalizationEngine localizationEngine;

    public void moveRobotToPos(Position destination, double maxAccel, double maxVelocity)
    {
        //Create a timer that we can use to track our time intervals.
        ElapsedTime timer = new ElapsedTime();
        Position initialPos = localizationEngine.getCurrentPosition();
        Position lastPos = initialPos;
        double currentDisplacement = 0;

        //Calculate displacement (S) between current position and desired position.
        double S = Math.hypot(Math.abs(destination.y - initialPos.y), Math.abs(destination.x - initialPos.x));

        //Calculate phase times and displacements.
        double accelPhaseTime = maxVelocity / maxAccel;
        double accelDisplacement = (0.5 * maxAccel * Math.pow(accelPhaseTime,2));
        double cruiseDisplacement = S - 2 * (accelDisplacement);
        double cruisePhaseTime = accelPhaseTime + (cruiseDisplacement / maxVelocity);
        double decelPhaseTime = cruisePhaseTime + accelPhaseTime;

        double targetPos = 0;

        //Loop until position has been reached
        while (localizationEngine.currentPosition != destination)
        {
            if (timer.seconds() < accelPhaseTime) //Use accel displacement formula if we are in accel phase
            {
                targetPos = 0.5 * maxAccel * Math.pow(timer.seconds(), 2);
            } else if (timer.seconds() > accelPhaseTime && timer.seconds() < decelPhaseTime) //Use cruise displacement formula if we are in cruise phase
            {
                targetPos = accelDisplacement + (maxVelocity * (timer.seconds() - accelPhaseTime));
            } else if (timer.seconds() > cruisePhaseTime) //Use decel displacement formula if we are in decel phase
            {
                double cruisePhaseT = timer.seconds() - cruisePhaseTime - accelPhaseTime;
                targetPos = accelDisplacement + cruiseDisplacement + ((maxVelocity * cruisePhaseT) - 0.5 * maxAccel * Math.pow(cruisePhaseT, 2));
            }

            //Since motion profile is 1-D, figure out how far we have travelled in terms of displacement and then calculate error and feed to PID.
            Position currentPos = localizationEngine.getCurrentPosition();
            currentDisplacement += Math.hypot(Math.abs(currentPos.y - lastPos.y), Math.abs(currentPos.y - lastPos.x));
            calculateMotorPowersUsingPID(targetPos - currentDisplacement, destination);
            lastPos = localizationEngine.getCurrentPosition();
        }
    }

    private void calculateMotorPowersUsingPID(double error, Position destination)
    {

    }
}

