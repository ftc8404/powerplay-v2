package org.quixilver8404.skystone.hardware.sensor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.quixilver8404.skystone.hardware.motor.BaseMotor;

/**
 * Encoder that supports getting delta readings
 */
public class DeltaEncoder extends Encoder {

    private int lastEncoderReading = 0;

    /**
     * For an encoder with no active motor on the same port
     */
    public DeltaEncoder(String deviceName, DcMotorSimple.Direction direction, HardwareMap hwMap) {
        super(deviceName, direction, hwMap);
    }

    /**
     * For an encoder with an active motor on the same port.
     */
    public DeltaEncoder(BaseMotor motor, DcMotorSimple.Direction direction) {
        super(motor, direction);
    }

    /**
     * For an encoder with an active motor on the same port.
     * The direction will match the motor's direction
     */
    public DeltaEncoder(BaseMotor motor) {
        super(motor);
    }

    /**
     * Zeros out the encoder, also adjusting the effect of runToPosition and also
     * resetting the last position for getDeltaPosition()
     */
    public void reset() {
        super.reset();
        lastEncoderReading = encoderZeroPosition;
    }

    /**
     * gets the change in encoder position since the last this object's construction,
     * reset(), or the last call to getDeltaPosition()
     */
    public int getDeltaPosition() {
        int curEncoderPosition = getEncoderPosition();
        int deltaPosition = curEncoderPosition - lastEncoderReading;
        lastEncoderReading = curEncoderPosition;
        return deltaPosition;
    }
}
