package com.chsrobotics.ftccore.hardware;

import com.chsrobotics.ftccore.engine.navigation.control.PID;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * The HardwareManager class centralizes all robot hardware.
 * Specifying this interface is required for most functions throughout FTCCore.
 */
public class HardwareManager {
    /**
     * The hardware map object used access hardware configuration to create hardware objects.
     */
    public final HardwareMap hardwareMap;

    public double offset = 0;

    /**
     * Four component array containing the four drive motors. Modifying this variable is extremely dangerous.
     */
    public DcMotorEx[] driveMotors;
    /**
     * The IMU. Modifying this variable is extremely dangerous.
     */
    public BNO055IMU imu;

    public final boolean debugMode;

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

    public boolean useCV;

    /**
     * Main PID controllers that the Nav engine will draw on;
     */
    public PID linearCtrler;
    public PID rotCtrler;

    /**
     * The LinearOpMode
     */
    public LinearOpMode opMode;

    public double linearSpeed = 1, rotSpeed = 1;
    public double IMUReset = 0;

    public boolean imuLocalEnabled = true;

    /**
     * Creates a hardware management interface and initializes all the hardware as specified by the configuration.
     * @param config The robot configuration. This can be created through the ConfigBuilder class.
     * @param hardware The hardware map object used access hardware configuration to create hardware objects.
     */
    public HardwareManager(Config config, HardwareMap hardware)
    {
        hardwareMap = hardware;
        debugMode = config.debugMode;
        initializeDriveMotors(config);
        initializeIMU(config);
        initializeAccessories(config);
        if (config.linearCoeffs != null && config.rotCoeffs != null)
            initializePID(config);

        initializeLinearOpMode(config);

        if (config.useCV)
            this.useCV = config.useCV;

        linearSpeed = config.linearSpeed;
        rotSpeed = config.rotSpeed;
    }

    private void initializeDriveMotors(Config config)
    {
        driveMotors = new DcMotorEx[4];
        for (int i = 0; i < 4; i++)
        {
            driveMotors[i] = (DcMotorEx) hardwareMap.dcMotor.get(config.driveMotors[i]);
            driveMotors[i].setDirection(DcMotorSimple.Direction.FORWARD);
            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void initializeIMU(Config config)
    {
        offset = config.offset;

        if (!imuLocalEnabled)
            return;

        imu = hardwareMap.get(BNO055IMU.class, config.imu);

        if (config.params == null)
        {
            BNO055IMU.Parameters params = new BNO055IMU.Parameters();
            params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imu.initialize(params);
        } else
            imu.initialize(config.params);
    }

    private void initializeAccessories(Config config)
    {
        if (config.accessories.size() == 0) {
            accessoryCameras = new WebcamName[0];
            accessoryMotors = new DcMotorEx[0];
            accessoryServos = new Servo[0];
            return;
        }

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

        accessoryMotors = new DcMotorEx[motors];
        accessoryServos = new Servo[servos];
        accessoryCameras = new WebcamName[cameras];


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

                    accessoryMotors[motorIndex].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

    /**
     * Checks to see if navigation by drive motor encoders is enabled.
     */
    public boolean isNavEnabled() {
        return driveMotors != null;
    }

    /**
     * Checks to see if navigation by IMU is enabled.
     */
    public boolean isImuLocalEnabled() {
        return imu != null;
    }

    private void initializePID(Config config)
    {
        linearCtrler = new PID(config.linearCoeffs);
        rotCtrler = new PID(config.rotCoeffs);
    }

    private void initializeLinearOpMode(Config config)
    {
        opMode = config.opMode;
    }


    public DcMotorEx getLeftFrontMotor() {
        return driveMotors[0];
    }
    public DcMotorEx getRightFrontMotor() {
        return driveMotors[1];
    }
    public DcMotorEx getRightBackMotor() {
        return driveMotors[2];
    }
    public DcMotorEx getLeftBackMotor() { return driveMotors[3]; }
    public DcMotorEx getLiftMotor() { return accessoryMotors[0]; }
    public WebcamName getWebcam() { return accessoryCameras[0]; }
}
