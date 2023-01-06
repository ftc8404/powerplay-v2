package org.quixilver8404.skystone.hardware.sensor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.quixilver8404.skystone.hardware.HardwareDevice;
import org.quixilver8404.skystone.util.measurement.Distance;

public class DistanceSensor extends HardwareDevice {
    private com.qualcomm.robotcore.hardware.DistanceSensor distanceSensor;

    public DistanceSensor(String deviceName, HardwareMap hwMap) {
        super(deviceName);
        distanceSensor = hwMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, deviceName);
    }

    public Distance getDistance() {
        return new Distance(distanceSensor.getDistance(DistanceUnit.METER), Distance.Unit.METERS);
    }
}
