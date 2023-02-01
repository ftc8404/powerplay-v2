package org.quixilver8404.powerplay.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Represents a motor that supports moving to a position using an internal PID loop
 * and will reset the motor's encoder
 */
public class EncoderPositionMotor extends EncoderMotor {

    private double prevRunPower = 0;

    protected final DcMotorEx motorEx;
    private int prevTargetPosition = 0;

    public EncoderPositionMotor(String deviceName, DcMotorSimple.Direction direction, HardwareMap hwMap) {
        super(deviceName, direction, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, hwMap);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorEx = (DcMotorEx) motor;
    }

    /**
     * The input targetPosition is relative to the zero position after any resets to the encoder
     */
    public void runToPosition(int targetPosition) {
        if (targetPosition != prevTargetPosition) {
            prevTargetPosition = targetPosition;
            motor.setTargetPosition(encoder.convertPositionToRaw(targetPosition));
        }
    }

    /**
     * Calls reset() on the internal encoder.
     * Zeros out the encoder, also adjusting the effect of runToPosition
     */
    public void reset() {
        encoder.reset();
    }

    /**
     * Set how fast the motor should move to reach the target position.
     * An absolute value followed by a clipping to the range [0,1] is applied to the input power.
     */
    public void setPower(double power) {
        double scaledPower = Math.abs(power);
        if (scaledPower > 1) {
            scaledPower = 1;
        }
        // Prevent unnecessary hardware I/O
        if (scaledPower != prevRunPower) {
            prevRunPower = scaledPower;
            motor.setPower(scaledPower);
        }
    }

    public void setVelocityPIDF(double p, double i, double d, double f) {
        motorEx.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void setPositionPIDF(double p) {
        motorEx.setPositionPIDFCoefficients(p);
    }

    public void setPositionTolerance(int encoderTicks) {
        motorEx.setTargetPositionTolerance(encoderTicks);
    }
}
