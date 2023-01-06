package org.quixilver8404.skystone.hardware.servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class PositionServo extends BaseServo {

    private final ServoImplEx servoEx;

    private double prevPosition = -1;

    private boolean isPowered = false;

    public PositionServo(String deviceName, Servo.Direction direction, HardwareMap hwMap) {
        super(deviceName, direction, hwMap);
        servoEx = (ServoImplEx) servo;
    }

    public PositionServo(String deviceName, HardwareMap hwMap) {
        this(deviceName, Servo.Direction.FORWARD, hwMap);
    }

    public PositionServo(String deviceName, Servo.Direction direction, double pwmLower, double pwmUpper, HardwareMap hwMap) {
        this(deviceName, direction, hwMap);
        servoEx.setPwmRange(new PwmControl.PwmRange(pwmLower, pwmUpper));
    }

    public PositionServo(String deviceName, double pwmLower, double pwmUpper, HardwareMap hwMap) {
        this(deviceName, Servo.Direction.FORWARD, pwmLower, pwmUpper, hwMap);
    }

    /**
     * This will also enable the servo if it is currently disabled.
     */
    public void setPosition(double position) {
        double scaledPosition = position;
        if (scaledPosition > 1) {
            scaledPosition = 1;
        } else if (scaledPosition < 0) {
            scaledPosition = 0;
        }
        // Prevent unnecessary hardware I/O
        if (scaledPosition != prevPosition) {
            prevPosition = scaledPosition;
            servo.setPosition(scaledPosition);
        }
        if (!isPowered) {
            enable();
        }
    }

    public void disable() {
        if (isPowered) {
            servoEx.setPwmDisable();
            isPowered = false;
        }
    }

    public void enable() {
        if (!isPowered) {
            servoEx.setPwmEnable();
            isPowered = true;
        }
    }
}
