package com.chsrobotics.ftccore.hardware.config;

import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class FTCCoreConfiguration {
    public String[] driveMotors;
    public String imu;
    public ArrayList<Accessory> accessories;

    public FTCCoreConfiguration()
    {
        accessories = new ArrayList();

        driveMotors = new String[] {"leftFrontMotor, rightFrontMotor, rightBackMotor, leftBackMotor"};
    }

    /**
     Registers the four drive motors. These motors should be specified in a clockwise order, beginning from the left front motor.
     This is a required component for navigation.
     @param motors Array containing the motor names. Length should be 4
     */
    public FTCCoreConfiguration setDriveMotors(String... motors)
    {
        driveMotors = motors;
        return this;
    }

    /**
     Registers the imu for the localization system.
     This is a required component for the IMU setting on the localization. Setting an IMU is highly recommended.
     @param imu The name of the IMU.
     */
    public FTCCoreConfiguration setIMU(String imu)
    {
        this.imu = imu;
        return this;
    }

    /**
     * Registers an accessory device.
     @param accessory The accessory.
     */
    public FTCCoreConfiguration addAccessory(Accessory accessory)
    {
        accessories.add(accessory);
        return this;
    }
}
