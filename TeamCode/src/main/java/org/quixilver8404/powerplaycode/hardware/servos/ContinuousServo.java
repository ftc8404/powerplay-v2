package org.quixilver8404.powerplaycode.hardware.servos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.quixilver8404.powerplaycode.hardware.HardwareDevice;

public class ContinuousServo extends HardwareDevice{

    protected final Servo servo;
    public final double powerRange;
    protected double prevPower;

    public ContinuousServo(final String deviceName, final Servo.Direction direction, final double powerRange, final HardwareMap hwMap) {
        super(deviceName);
        servo = hwMap.servo.get(deviceName);
        servo.setDirection(direction);
        this.powerRange = powerRange;
        this.prevPower = 0;
    }

    public ContinuousServo(final String deviceName, final Servo.Direction direction, final HardwareMap hwMap) {
        this(deviceName, direction, 1, hwMap);
    }

    public void setPower(final double power) {
        final double powerThatWontMakeTheServoBreak = Math.min(Math.max(power, -1), 1);
        // Prevent unnecessary hardware I/O
        if (powerThatWontMakeTheServoBreak != prevPower) {
            prevPower = powerThatWontMakeTheServoBreak;
            servo.setPosition(0.5 + (powerThatWontMakeTheServoBreak * powerRange/2));
        }
    }

    public double getPrevPower() {
        return prevPower;
    }

    public double getPosition() {
        return servo.getPosition();
    }

}
