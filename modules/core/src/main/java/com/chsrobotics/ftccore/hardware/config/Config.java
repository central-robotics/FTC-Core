package com.chsrobotics.ftccore.hardware.config;

import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class Config {
    public String[] driveMotors;
    public String imu;
    public ArrayList<Accessory> accessories;
    public PIDCoefficients linearCoeffs;
    public PIDCoefficients rotCoeffs;
    public LinearOpMode opMode;

    public boolean debugMode;


    public static class Builder {
        private String[] driveMotors;
        private String imu;
        private ArrayList<Accessory> accessories;
        private PIDCoefficients linearCoeffs;
        private PIDCoefficients rotCoeffs;
        private LinearOpMode opMode;

        private boolean debugMode;



        /**
         Creates a new instance of the ConfigBuilder for setting up the robot hardware.
         */
        public Builder()
        {
            accessories = new ArrayList<>();
        }

        /**
         Registers the four drive motors. These motors should be specified in a clockwise order, beginning from the left front motor.
         This is a required component for navigation.
         @param m0 The left front motor.
         @param m1 The right front motor.
         @param m2 The right rear motor.
         @param m3 The left rear motor.
         */
        public Builder setDriveMotors(String m0, String m1, String m2, String m3)
        {
            driveMotors = new String[] {
                    m0, m1, m2, m3
            };

            return this;
        }

        /**
         * If debug mode is enabled, telemetry will be shown in the driver station for debug purposes.
         * @param mode Sets debug mode to true or false.
         * @return
         */
        public Builder setDebugMode(boolean mode) {
            debugMode = mode;
            return this;
        }

        /**
         Registers the imu for the localization system.
         This is a required component for the IMU setting on the localization. Setting an IMU is highly recommended.
         @param imu The name of the IMU.
         */
        public Builder setIMU(String imu)
        {
            this.imu = imu;

            return this;
        }

        /**
         * Registers an accessory device.
         @param accessory The accessory.
         */
        public Builder addAccessory(Accessory accessory)
        {
            accessories.add(accessory);

            return this;
        }

        /**
         * Registers the current OpMode
         * @param opMode the OpMode
         * @return
         */
        public Builder setOpMode(LinearOpMode opMode)
        {
            this.opMode = opMode;

            return this;
        }

        /**
         * Sets the gain values for the main PID controller
         * @param linearCoeffs coefficients for linear movements
         * @param rotCoeffs coefficients for rotational movements
         */
        public Builder setPIDCoefficients(PIDCoefficients linearCoeffs, PIDCoefficients rotCoeffs)
        {
            this.linearCoeffs = linearCoeffs;
            this.rotCoeffs = rotCoeffs;

            return this;
        }

        /**
         Derives a robot configuration from the configuration builder.
         */
        public Config build() {
            Config config = new Config();

            config.driveMotors = driveMotors;
            config.imu = imu;
            config.accessories = accessories;
            config.linearCoeffs = linearCoeffs;
            config.rotCoeffs = rotCoeffs;
            config.opMode = opMode;
            config.debugMode = debugMode;

            return config;
        }
    }
}
