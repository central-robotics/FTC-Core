package com.chsrobotics.ftccore.hardware.config;

import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class Config {
    public String[] driveMotors;
    public String imu;
    public ArrayList<Accessory> accessories;
}
