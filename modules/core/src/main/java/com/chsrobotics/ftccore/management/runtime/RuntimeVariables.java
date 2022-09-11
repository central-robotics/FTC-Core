package com.chsrobotics.ftccore.management.runtime;

public class RuntimeVariables {
    private static OpMode currentOpMode = OpMode.RED_RIGHT;

    /**
     * @return The starting alliance color and field position.
     */
    public static OpMode getCurrentOpMode() {
        return currentOpMode;
    }

    /**
     * Sets the current starting alliance color and field position
     */
    public static void setCurrentOpMode(OpMode currentOpMode) {

        RuntimeVariables.currentOpMode = currentOpMode;
    }

    private enum OpMode {
        RED_LEFT,
        RED_RIGHT,
        BLUE_LEFT,
        BLUE_RIGHT
    }
}
