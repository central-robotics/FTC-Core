package com.chsrobotics.ftccore.hardware.config.accessory;

public class Accessory {
    /**
     * Type of accessory.
     */
    public final AccessoryType accessoryType;
    /**
     * Configuration name of the accessory.
     */
    public final String name;

    /**
     * Creates a new accessory.
     * @param type Specifies the type of accessory.
     * @param name Specifies the configuration name of the accessory.
     */
    public Accessory(AccessoryType type, String name) {
        accessoryType = type;
        this.name = name;
    }
}
