package com.chsrobotics.ftccore.hardware.config;

import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
    public BNO055IMU.Parameters params;
    private double linearSpeed = 1, rotSpeed = 1;

    public boolean debugMode;


    public static class Builder {
        private String[] driveMotors;
        private String imu;
        private ArrayList<Accessory> accessories;
        private PIDCoefficients linearCoeffs;
        private PIDCoefficients rotCoeffs;
        private LinearOpMode opMode;
        private double linearSpeed = 1, rotSpeed = 1;
        private BNO055IMU.Parameters params;

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
         * Calculates distance per tick for accurate autonomous movement. If this is not done correct, robot will still navigate,
         * but distances will not be accurate.
         * @param encoderRes the resolution of the motor encoder. Usually found on website
         * @param wheelDiameterMM the diameter of the wheel in mm;
         */
        public Builder setWheelProperties(double encoderRes, int wheelDiameterMM)
        {
            //TODO
            return this;
        }

        public Builder setIMUSettings(BNO055IMU.Parameters params)
        {
            this.params = params;
            return this;
        }

        /**
         * Tunes tele-op power multipliers.
         * @param linearSpeed How fast the robot will move on the X, Y axis (0-1)
         * @param rotSpeed How fast the robot will move rotationally (0-1)
         */
        public Builder setTeleopValues(double linearSpeed, double rotSpeed)
        {
            this.linearSpeed = linearSpeed;
            this.rotSpeed = rotSpeed;
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
            config.linearSpeed = linearSpeed;
            config.rotSpeed = rotSpeed;
            config.params = params;

            return config;
        }
    }
}
