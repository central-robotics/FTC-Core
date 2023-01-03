package com.chsrobotics.ftccore.hardware.config;

import com.chsrobotics.ftccore.engine.navigation.path.Tolerances;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.util.ArrayList;

public class Config {
    public String[] driveMotors;
    public String imu;
    public ArrayList<Accessory> accessories;
    public PIDCoefficients linearCoeffs;
    public PIDCoefficients rotCoeffs;
    public PIDCoefficients profileCoeffs;
    public LinearOpMode opMode;
    public BNO055IMU.Parameters params;
    public DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

    public double offset;
    public double linearSpeed = 1, rotSpeed = 1;

    public boolean debugMode;
    public boolean useCV;
    public boolean thetaReversed;

    public float encoderRes;
    public double wheelDiameter;
    public double latWheelOffset;
    public double lonWheelOffset;
    public boolean useDegrees = false;

    public Tolerances lowPrecisionTolerances;
    public Tolerances mediumPrecisionTolerances;
    public Tolerances highPrecisionTolerances;

    public static class Builder {
        private String[] driveMotors;
        private String imu;

        private ArrayList<Accessory> accessories;
        private PIDCoefficients linearCoeffs;
        private PIDCoefficients rotCoeffs;
        private PIDCoefficients profileCoeffs;
        private LinearOpMode opMode;
        private double offset = 0;
        private double linearSpeed = 1, rotSpeed = 1;
        private BNO055IMU.Parameters params;
        public DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private float encoderRes;
        private double wheelDiameter;
        private double latWheelOffset;
        private double lonWheelOffset;

        private boolean debugMode;
        private boolean useDegrees = false;

        private boolean reversed = false;
        private boolean useCV = false;
        public boolean thetaReversed = false;

        private Tolerances lowPrecisionTolerances;
        private Tolerances mediumPrecisionTolerances = new Tolerances(15, 0.1);
        private Tolerances highPrecisionTolerances;


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

        public Builder setMotionProfilePIDCoeffs(PIDCoefficients coeffs)
        {
            profileCoeffs = coeffs;
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

        public Builder useDegrees(boolean mode)
        {
            useDegrees = mode;
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
         * @param wheelDiameter the diameter of the wheel in mm;
         */
        public Builder setOdometryWheelProperties(float encoderRes, double wheelDiameter, double latWheelOffset, double lonWheelOffset)
        {
            //TODO
            this.encoderRes = encoderRes;
            this.wheelDiameter = wheelDiameter;
            this.latWheelOffset = latWheelOffset;
            this.lonWheelOffset = lonWheelOffset;
            return this;
        }

        public Builder setNavigationTolerances(Tolerances tolerances)
        {
            this.mediumPrecisionTolerances = tolerances;
            return this;
        }

        public Builder setHighPrecisionTolerances(Tolerances tolerances)
        {
            this.highPrecisionTolerances = tolerances;
            return this;
        }

        public Builder setLowPrecisionTolerances(Tolerances tolerances) {
            this.lowPrecisionTolerances = tolerances;
            return this;
        }

        /**
         * Sets the imu parameters. Should be used based on where and how the REV control hub is oriented on the robot.
         * @param params User specified BNO055IMU parameters. For extra information, read FTC documentation.
         */
        public Builder setIMUSettings(BNO055IMU.Parameters params)
        {
            this.params = params;
            return this;
        }

        public Builder setMotorDirection(DcMotorSimple.Direction direction)
        {
            this.direction = direction;
            return this;
        }

        /**
         * If the orientation is inverted or modified in some way, IMUOffset will add a constant to add or subtract the calculated orientation
         * of the robot.
         * @param offset Offset value
         */
        public Builder setIMUOffset(double offset)
        {
            this.offset = offset;
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

        public Builder useCV()
        {
            useCV = true;
            return this;
        }

        public Builder reverseTheta()
        {
            thetaReversed = true;
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
            config.offset = offset;
            config.direction = direction;
            config.useCV = useCV;
            config.thetaReversed = thetaReversed;
            config.encoderRes = encoderRes;
            config.wheelDiameter = wheelDiameter;
            config.latWheelOffset = latWheelOffset;
            config.lonWheelOffset = lonWheelOffset;
            config.useDegrees = useDegrees;
            config.lowPrecisionTolerances = lowPrecisionTolerances;
            config.mediumPrecisionTolerances = mediumPrecisionTolerances;
            config.highPrecisionTolerances = highPrecisionTolerances;
            config.profileCoeffs = profileCoeffs;

            return config;
        }
    }
}
