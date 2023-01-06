package org.quixilver8404.skystone.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Represents a motor that can be made to move at a power
 * and handles redundant I/O requests
 */
public abstract class PowerMotor extends BaseMotor {

    private double prevPower = 0;

    PowerMotor(String deviceName, DcMotorSimple.Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior, HardwareMap hwMap) {
        super(deviceName, direction, hwMap);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    PowerMotor(String deviceName, DcMotorSimple.Direction direction, HardwareMap hwMap) {
        this(deviceName, direction, DcMotor.ZeroPowerBehavior.BRAKE, hwMap);
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
            motor.setPower(scaledPower);
        }
    }

    public double getPower() {
        return motor.getPower();
    }
}
