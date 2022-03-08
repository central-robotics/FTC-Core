package com.chsrobotics.ftccore.hardware;

import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * The HardwareManager class centralizes all robot hardware.
 * Specifying this interface is required for most functions throughout FTCCore.
 */
public class HardwareManager {
    /**
     * If no drive motors were specified in the configuration, this provides an easier way to tell if navigation should be used or not.
     */
    public final boolean navEnabled;
    /**
     * If no IMU was specified, this provides an easier way to tell if IMU-based orientation should be used within the localization engine.
     */
    public final boolean imuLocalEnabled;
    /**
     * The hardware map object used access hardware configuration to create hardware objects.
     */
    public final HardwareMap hardwareMap;

    /**
     * Four component array containing the four drive motors. Modifying this variable is extremely dangerous.
     */
    public DcMotor[] driveMotors;
    /**
     * The IMU. Modifying this variable is extremely dangerous.
     */
    public BNO055IMU imu;

    /**
     * Array containing accessory motors.
     */
    public DcMotorEx[] accessoryMotors;
    /**
     * Array containing accessory servos;
     */
    public Servo[] accessoryServos;
    /**
     * Array containing accessory cameras;
     */
    public WebcamName[] accessoryCameras;

    /**
     * Creates a hardware management interface and initializes all the hardware as specified by the configuration.
     * @param config The robot configuration. This can be created through the ConfigBuilder class.
     * @param hardware The hardware map object used access hardware configuration to create hardware objects.
     */
    public HardwareManager(Config config, HardwareMap hardware)
    {
        hardwareMap = hardware;

        if (config.imu == null)
            imuLocalEnabled = false;
        else
            imuLocalEnabled = true;

        if (config.driveMotors == null)
            navEnabled = false;
        else
            navEnabled = true;

        initializeHardware(config);
    }

    private void initializeHardware(Config config)
    {
        initializeDriveMotors(config);
        initializeIMU(config);
        initializeAccessories(config);
    }

    private void initializeDriveMotors(Config config)
    {
        if (!navEnabled)
            return;

        for (int i = 0; i < 4; i++)
        {
            driveMotors[i] = hardwareMap.dcMotor.get(config.driveMotors[i]);
            driveMotors[i].setDirection(DcMotorSimple.Direction.FORWARD);
            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void initializeIMU(Config config)
    {
        if (!imuLocalEnabled)
            return;

        imu = hardwareMap.get(BNO055IMU.class, config.imu);

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(params);
    }

    private void initializeAccessories(Config config)
    {
        if (config.accessories.size() == 0)
            return;

        int motors = 0;
        int servos = 0;
        int cameras = 0;

        for (Accessory a : config.accessories)
        {
            switch (a.accessoryType)
            {
                case MOTOR:
                    motors++;
                    break;
                case SERVO:
                    servos++;
                    break;
                case WEBCAM:
                    cameras++;
                    break;
                default:
                    break;
            }
        }

        if (motors > 0) {
            accessoryMotors = new DcMotorEx[motors];
        }

        if (servos > 0) {
            accessoryServos = new Servo[servos];
        }

        if (cameras > 0) {
            accessoryCameras = new WebcamName[cameras];
        }

        int motorIndex = 0;
        int servoIndex = 0;
        int webcamIndex = 0;

        for (int i = 0; i < config.accessories.size(); i++)
        {
            switch (config.accessories.get(i).accessoryType)
            {
                case MOTOR:
                    accessoryMotors[motorIndex] = hardwareMap.get(DcMotorEx.class,
                            config.accessories.get(i).name);

                    accessoryMotors[motorIndex].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    accessoryMotors[motorIndex].setDirection(DcMotorSimple.Direction.FORWARD);
                    accessoryMotors[motorIndex].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    accessoryMotors[motorIndex].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    motorIndex++;
                    break;
                case SERVO:
                    accessoryServos[servoIndex] = hardwareMap.get(Servo.class,
                            config.accessories.get(i).name);
                    servoIndex++;
                    break;
                case WEBCAM:
                    accessoryCameras[webcamIndex] = hardwareMap.get(WebcamName.class,
                            config.accessories.get(i).name);
                    webcamIndex++;
                    break;
            }
        }

    }
}
