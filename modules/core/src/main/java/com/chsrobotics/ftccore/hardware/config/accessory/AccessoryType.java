package com.chsrobotics.ftccore.hardware.config.accessory;

/**
 * Specifies the type of accessory being registered to the robot configuration.
 */
public enum AccessoryType {
    /**
     * Specifies that the accessory being registered is a DC motor.
     */
    MOTOR,
    /**
     * Specifies that the accessory being registered is a servo.
     */
    SERVO,
    /**
     * Specifies that the accessory being registered is a standard webcam for use with OpenCV or Vuforia.
     */
    WEBCAM,
    /**
     * Specifies that the accessory being registered is a Intel T265 camera for use with the localization engine.
     */
    T265CAM,
    /**
     * Odometry pod
     */
    ODOMETRY_POD,
    /**
     * Continuous rotation servo
     */
    CONTINUOUS_SERVO
}
