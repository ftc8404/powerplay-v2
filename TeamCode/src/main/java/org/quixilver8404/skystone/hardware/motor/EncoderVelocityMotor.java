package org.quixilver8404.skystone.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Represents a motor that supports supports using a PID loop to move at a proportional velocity
 */
public class EncoderVelocityMotor extends EncoderMotor {

    private double prevVelocity = 0;

    protected final DcMotorEx motorEx;

    public EncoderVelocityMotor(String deviceName, DcMotorSimple.Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior, HardwareMap hwMap) {
        super(deviceName, direction, zeroPowerBehavior, DcMotor.RunMode.RUN_USING_ENCODER, hwMap);
        motorEx = (DcMotorEx) motor;
    }

    public EncoderVelocityMotor(String deviceName, DcMotorSimple.Direction direction, HardwareMap hwMap) {
        this(deviceName, direction, DcMotor.ZeroPowerBehavior.BRAKE, hwMap);
    }

    public void setVelocityPIDF(double p, double i, double d, double f) {
        motorEx.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void setVelocity(double ticksPerSecond) {
        if (ticksPerSecond != prevVelocity) {
            prevVelocity = ticksPerSecond;
            motorEx.setVelocity(ticksPerSecond);
        }
    }
}
