package org.quixilver8404.powerplaycode.hardware.servos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.quixilver8404.powerplaycode.hardware.HardwareDevice;

public class BaseServo extends HardwareDevice{

    protected final ServoImplEx servo;
    protected double lastPosition;

    public BaseServo(final String deviceName, final Servo.Direction direction, final HardwareMap hwMap) {
        super(deviceName);
        servo = (ServoImplEx) hwMap.servo.get(deviceName);
        servo.setDirection(direction);
        lastPosition = Double.NaN;
    }

    public void setPosition(final double position) {
        if (position != lastPosition) {
            servo.setPosition(position);
            lastPosition = position;
        }
    }

    public double getPosition() {
        return servo.getPosition();
    }

    public void disable() {
        servo.setPwmDisable();
    }

    public void enable() {
        servo.setPwmEnable();
    }

}
