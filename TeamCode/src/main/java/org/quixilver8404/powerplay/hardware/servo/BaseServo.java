package org.quixilver8404.powerplay.hardware.servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.quixilver8404.powerplay.hardware.HardwareDevice;

abstract class BaseServo extends HardwareDevice {

    protected final Servo servo;

    BaseServo(String deviceName, Servo.Direction direction, HardwareMap hwMap) {
        super(deviceName);
        servo = hwMap.servo.get(deviceName);
        servo.setDirection(direction);
    }
}
