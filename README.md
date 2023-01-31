![enter image description here](https://i.imgur.com/NFUtYb1.png)\
\
Designed by Daniel Huinda, Garrison Taylor, and Zack Murray.\
Central High School Robotics in Springfield, Missouri\
\
\
##### Table of Contents  \
  [Disclaimer](#disclaimer-)\
  [Introduction](#introduction)\
  [Installation](#installation)\
  [Usage](#usage)\
    + [Hardware Configuration](#hardware-configuration)\
\
\
### Disclaimer:\
The GitBook and other documentation assumes basic knowledge of Java. As this library was developed with inclusion in mind, there is also a video series which walks through exactly how to install all needed prerequisites, including installing Android Studio, setting up the project, connecting the project to the robot, installing the library, as well as basic usage of the library to get you up and running! \
\
Video Series:\
\
 ## Introduction\
Welcome to FTC Core! FTC Core is a multipurpose library that allows both beginner and advanced FTC teams to design advanced teleop and autonomous systems in under 30, very, very, simple lines of code. Inspired from libraries like FTCLib and Roadrunner, we wanted to develop a library that integrated extensive navigation and localization systems while providing plenty of space for teams to extend as much functionality as they need to execute their strategies. \
\
## Installation\
Below is a detailed guide which describes how to install FTC Core into your project. \
\
## Usage\
### Hardware Configuration\
A unique feature of FTC Core is that most hardware initialization is handled automatically. However, to achieve this, the program makes certain assumptions about the configuration of the robot. Configuration should be nearly identical between teleop and autonomous OpModes. \
\
In the current configuration of the library, only 4 wheel holonomic drive setups are supported. These can be both X-Drive, Mecanum Drive, or any other drive which functionally works the same way.  \
\
In a LinearOpMode, all FTC Core instructions should be under the *runOpMode* method, beginning with configuration. Once built, the *Config* object is supplied to the *HardwareManager* which manages the rest of the hardware components. \
\
FTC Core uses the builder format to encourage simplicity, including the *Config* class. The following details how to configure the robot:\
\
    Config config = new Config.Builder()  \
		.setDriveMotors("m0", "m1", "m2", "m3")\
		.setIMU("imu")  \
		.addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))  \
	    .setOpMode(this)  \
	    .build();\
A *HardwareManager* should then be created using this configuration:\
\
    HardwareManager manager = new HardwareManager(config, hardwareMap);\
The following table clarifies the usage of *Config* options.\
|Config Option| Notes|\
|--|--|\
| .setDriveMotors| Drive motors are in the format of the front left, front right, back right, and back left motors respectively. This configuration option is necessary for both navigation and localization if applicable.  |\
|.setIMU| "imu" is the default name of the IMU; however, it may be different based on different robot configurations. This feature is necessary for navigation and localization. \
|.addAccessory|Not necessary for any function; however, can be used to extend custom functionality in the code. See Actions.\
|.setOpMode|Required. Should be supplied with the *this* keyword.\
|.build| Required for completion of configuration setup.\
\
### Autonomous Configuration\
For autonomous configuration, FTC Core utilizes the *Pipeline* class. Using the *Pipeline* class allows the user to configure a navigation path and a sequence of user defined actions. This is then executed and FTC Core handles the rest. }