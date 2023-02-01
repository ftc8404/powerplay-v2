package org.quixilver8404.powerplay.hardware.servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ContinuousServo extends BaseServo {
    private double prevPower = -2;

    public ContinuousServo(String deviceName, Servo.Direction direction, HardwareMap hwMap) {
        super(deviceName, direction, hwMap);
    }

    public ContinuousServo(String deviceName, Servo.Direction direction, double range, HardwareMap hwMap) {
        super(deviceName, direction, hwMap);
        servo.scaleRange(0.5 - range, 0.5 + range);
    }

    public ContinuousServo(String deviceName, HardwareMap hwMap) {
        this(deviceName, Servo.Direction.FORWARD, hwMap);
    }

    public ContinuousServo(String deviceName, double range, HardwareMap hwMap) {
        this(deviceName, Servo.Direction.FORWARD, range, hwMap);
    }

    public void setPower(double power) {
        double scaledPower = power;
        if (scaledPower > 1) {
            scaledPower = 1;
        } else if (scaledPower < -1) {
            scaledPower = -1;
        }
        // Prevent unnecessary hardware I/O
        if (scaledPower != prevPower) {
            prevPower = scaledPower;
            servo.setPosition(0.5 + (scaledPower / 2.0));
        }
    }
}
