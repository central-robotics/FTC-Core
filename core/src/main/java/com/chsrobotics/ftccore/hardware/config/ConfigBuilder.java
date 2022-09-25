package com.chsrobotics.ftccore.hardware.config;

import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class ConfigBuilder {
    private String[] driveMotors;
    private String imu;
    private ArrayList accessories;

    /**
     Creates a new instance of the ConfigBuilder for setting up the robot hardware.
     */
    public ConfigBuilder()
    {
        accessories = new ArrayList();
    }

    /**
    Registers the four drive motors. These motors should be specified in a clockwise order, beginning from the left front motor.
     This is a required component for navigation.
     @param m0 The left front motor.
     @param m1 The right front motor.
     @param m2 The right rear motor.
     @param m3 The left rear motor.
     */
    public ConfigBuilder setDriveMotors(String m0, String m1, String m2, String m3)
    {
        driveMotors = new String[] {
                m0, m1, m2, m3
        };

        return this;
    }

    /**
     Registers the imu for the localization system.
     This is a required component for the IMU setting on the localization. Setting an IMU is highly recommended.
     @param imu The name of the IMU.
     */
    public ConfigBuilder setIMU(String imu)
    {
        this.imu = imu;

        return this;
    }

    /**
     * Registers an accessory device.
     @param accessory The accessory.
     */
    public ConfigBuilder addAccessory(Accessory accessory)
    {
        accessories.add(accessory);

        return this;
    }

    /**
     Derives a robot configuration from the configuration builder.
     */
    public Config createConfig() {
        Config config = new Config();

        config.driveMotors = driveMotors;
        config.imu = imu;
        config.accessories = accessories;

        return config;
    }
}
