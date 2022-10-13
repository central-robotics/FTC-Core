package com.chsrobotics.ftccore;

import android.content.Context;
import androidx.test.platform.app.InstrumentationRegistry;
import androidx.test.ext.junit.runners.AndroidJUnit4;

import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import org.junit.Test;
import org.junit.runner.RunWith;

import static org.junit.Assert.*;

/**
 * Instrumented test, which will execute on an Android device.
 *
 * @see <a href="http://d.android.com/tools/testing">Testing documentation</a>
 */
@RunWith(AndroidJUnit4.class)
public class ExampleInstrumentedTest {
    @Test
    public void useAppContext() {
        // Context of the app under test.
        Context appContext = InstrumentationRegistry.getInstrumentation().getTargetContext();
        assertEquals("com.chsrobotics.ftccore", appContext.getPackageName());

        Config configuration = new Config.Builder()
                .setDriveMotors("","","","")
                .setIMU("imu")
                .addAccessory(new Accessory(AccessoryType.MOTOR, "name"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "servo"))
                .addAccessory(new Accessory(AccessoryType.WEBCAM, "camera"))
                .build();

        HardwareManager hardwareManager = new HardwareManager(configuration, null);
    }
}