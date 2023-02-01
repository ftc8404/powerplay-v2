package org.quixilver8404.powerplay.hardware.sensor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.quixilver8404.powerplay.hardware.HardwareDevice;
import org.quixilver8404.powerplay.hardware.motor.BaseMotor;

/**
 * Represents an encoder that may be independent from a motor on the same port number
 */
public class Encoder extends HardwareDevice {

    private final DcMotorEx encoderMotor;
    private final boolean isFlipped; // sometimes, the encoder isn't linked to the motor

    protected int encoderZeroPosition;

    protected int prevRawPosition = 0;

    /**
     * For an encoder with no active motor on the same port
     */
    public Encoder(String deviceName, DcMotor.Direction direction, HardwareMap hwMap) {
        super(deviceName);
        encoderMotor = (DcMotorEx) hwMap.dcMotor.get(deviceName);
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderMotor.setDirection(direction);
        encoderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        isFlipped = false;
        reset();
    }

    /**
     * For an encoder with an active motor on the same port.
     */
    public Encoder(BaseMotor motor, DcMotor.Direction direction) {
        super(motor.getDeviceName());
        encoderMotor = (DcMotorEx) motor.getInternalMotor();
        // flip the direction if the specified direction does not match the motor's direction
        isFlipped = direction != encoderMotor.getDirection();
        reset();
    }

    /**
     * For an encoder with an active motor on the same port.
     * The direction will match the motor's direction
     */
    public Encoder(BaseMotor motor) {
        super(motor.getDeviceName());
        encoderMotor = (DcMotorEx) motor.getInternalMotor();
        isFlipped = false;
        reset();
    }

    public int getEncoderPosition() {
        return getRawPosition() - encoderZeroPosition;
    }

    private int getRawPosition() {
        try {
            int position = encoderMotor.getCurrentPosition();
            if (isFlipped) {
                position *= -1;
            }
            prevRawPosition = position;
            return position;
        } catch (NullPointerException e) {
            return prevRawPosition;
        }
    }

    /**
     * Gets the velocity in ticks per second
     */
    public double getEncoderVelocity() {
        double velocity = encoderMotor.getVelocity();
        if (isFlipped) {
            velocity *= -1;
        }
        return velocity;
    }

    /**
     * Zeros out the encoder, also adjusting the effect of runToPosition
     */
    public void reset() {
        encoderZeroPosition = getRawPosition();
    }

    /**
     * Converts the input position to a raw position that bypasses any effects of reset()
     */
    public int convertPositionToRaw(int position) {
        return encoderZeroPosition + position;
    }
}
