# Configuration

Configuration in FTC-Core is handled by the Config class which houses a variety of useful options. Once an instance of Config has been created, it is passed as an argument to an instance of HardwareManager to reflect those options in the robot hardware.

## Builder

The configuration builder allows programmers to pass an arbitrary number of configurable options to the hardware manager. To use the builder, programmers must define each new instance of Config as a Config Builder. All builders must terminate with the build() method.

```java
// Example Config
// 
// This example includes the bare minimum options required for FTC-Core
// to run on most robots. Additional options are listed below.

Config config = new Config.Builder()
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

For teams that need access to options that aren't built into the config class, we leave every device accessible via the HardwareManager. For instance, the following code can be used to set the zero-power behavior of the 4 drive motors:

```java
// HardwareManager can be used to further configure individual devices

manager.driveMotors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
manager.driveMotors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
manager.driveMotors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
manager.driveMotors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
```

## Options Index

\*: required

### Localization/Navigation

#### Drive Motors (\*)

Currently, FTC-Core supports 4 drive motors for mecanum/holonomic drive, which can be configured as follows:

```java
setDriveMotors("leftFront", "rightFront", "rightBack", "leftBack");
```

#### IMU (\*)

FTC-Core uses the built-in IMU to track the orientation of the robot. For this to work, programmers need to provide the name of the IMU as configured in the Driver Station.

```java
// By default, the IMU is just called "imu" in configuration

setIMU("imu");
```

#### Odometry Wheels (\*)

As of right now, the setOdometryWheelProperties is used to set the properties of all localizing wheelsets, including drive wheels and odometry pods. This will change once more localizers are supported and localization fusion has been implemented.

For drive wheel localization, x\_offset and y\_offset would be 0.&#x20;

```java
setOdometryWheelProperties(encoder_ticks_per_rev, wheel_dia, x_offset, y_offset);
```

#### PID Controllers (\*)

FTC-Core uses two PID controllers; one for linear motion and one for rotation. Each controller must be initialized with its own set of parameters as follows:

```java
// The coefficients we use work for movements measured in mm,
// rotations measured in RAD, 60mm wheels, and 312 rpm goBilda
// 5303 motors.

setPIDCoefficients(
        new PIDParams(4.5, 0.0002, 0),     // Params for linear mvt
        new PIDParams(750, 0.03, 0)        // Params for rotation
);
```

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

