package org.quixilver8404.skystone.hardware.sensor;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.quixilver8404.skystone.hardware.HardwareDevice;

/**
 * Represents an analog sensor
 */
public class AnalogSensor extends HardwareDevice {

    private final AnalogInput analogInput;
    private final double minVoltage; // in volts
    private final double maxVoltage; // in volts

    public AnalogSensor(String deviceName, HardwareMap hwMap) {
        super(deviceName);
        analogInput = hwMap.analogInput.get(deviceName);
        minVoltage = 0;
        maxVoltage = analogInput.getMaxVoltage();
    }

    /**
     * Gets the reading as a fraction of its voltage range fom 0 to 1
     */
    public double getReading() {
        double rawVoltage = analogInput.getVoltage();
        return (rawVoltage - minVoltage) / (maxVoltage - minVoltage);
    }
}
