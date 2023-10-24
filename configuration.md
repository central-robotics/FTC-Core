# Configuration

Configuration in FTC-Core is handled by the Config class which houses a variety of useful options. Once an instance of Config has been created, it is passed as an argument to an instance of HardwareManager to reflect those options in the robot hardware.

## Builder

The configuration builder allows programmers to pass an arbitrary number of configurable options to the hardware manager. To use the builder, programmers must define each new instance of Config as a Config Builder. All builders must terminate with the build() method.

```
// Example Config
// 
// This example includes the bare minimum options required for FTC-Core
// to run on most robots. Additional options are listed below.

Config config = new Config.Builder()
        // Drive motors MUST be declared in this order for navigation
        // to work properly
    .setDriveMotors("leftFront", "rightFront", "rightBack", "leftBack")
        // These properties are usable for GoBilda 5203 motors with 96mm
        // mecanum or omni-wheels (soon to be deprecated)
    .setOdometryWheelProperties(537.7, 60, 0, 0)
    .setOpMode(this)
    .setIMU("imu")
    .setPIDCoefficients(
        new PIDParams(4.5, 0.0002, 0),     // Params for linear mvt
        new PIDParams(750, 0.03, 0)        // Params for rotation
    )
    .setNavigationTolerances(new Tolerances(45, 0.15))
    .build();

HardwareManager manager = new HardwareManager(config, hardwareMap);
```

## HardwareManager

For teams that need access to options that aren't built into the config class, we leave each and every device accessible via the HardwareManager. For instance, the following code can be used to set the zero-power behavior of the 4 drive motors:

```
// HardwareManager can be used to further configure individual devices

manager.driveMotors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
manager.driveMotors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
manager.driveMotors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
manager.driveMotors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
```

## Options Index

### Localization/Navigation

#### Drive Motors

#### IMU

#### Odometry Wheels

#### PID Controllers

#### Speed Limits

#### Precision

#### Motion Profiling

### Accessories

#### Motors

#### Servos

#### Sensors

#### Camera

### Internal

#### OpMode

#### Debug

###

