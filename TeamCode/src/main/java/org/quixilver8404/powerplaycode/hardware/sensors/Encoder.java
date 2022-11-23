package org.quixilver8404.powerplaycode.hardware.sensors;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.quixilver8404.powerplaycode.hardware.HardwareDevice;
import org.quixilver8404.powerplaycode.hardware.motors.BaseMotor;

public class Encoder extends HardwareDevice {

    protected final LynxModule revHub;
    protected final DcMotorEx encoderMotor;
    protected final boolean isFlipped; // sometimes, the encoder isn't linked to the motor
    protected int encoderZeroPosition;
    protected int prevRawPosition = 0;
    protected int lastEncoderReading = 0;

    public Encoder(final String deviceName, final DcMotor.Direction direction, final LynxModule revHub, final HardwareMap hwMap) {
        super(deviceName);
        this.revHub = revHub;
        encoderMotor = (DcMotorEx) hwMap.dcMotor.get(deviceName);
        encoderMotor.setDirection(direction);
        isFlipped = false;
//        reset();
    }

    public Encoder(final BaseMotor motor, final DcMotor.Direction direction, final LynxModule revHub) {
        super(motor.getDeviceName());
        this.revHub = revHub;
        encoderMotor = (DcMotorEx) motor.getInternalMotor();
        // flip the direction if the specified direction does not match the motor's direction
        isFlipped = direction != encoderMotor.getDirection();
//        reset();
    }

    public Encoder(final BaseMotor motor, final LynxModule revHub) {
        super(motor.getDeviceName());
        this.revHub = revHub;
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
            e.printStackTrace();
            return prevRawPosition;
        }
    }

    public double getEncoderVelocity() {
        double velocity = encoderMotor.getVelocity();
        if (isFlipped) {
            velocity *= -1;
        }
        return velocity;
    }

    public void reset() {
        revHub.getBulkData();
        encoderZeroPosition = getRawPosition();
        lastEncoderReading = 0;
    }

    public void setPosition(final int position) {
        revHub.getBulkData();
        encoderZeroPosition = getRawPosition() - position;
        lastEncoderReading = position;
    }

    /**
     * Converts the input position to a raw position that bypasses any effects of reset()
     */
    public int convertPositionToRaw(int position) {
        return encoderZeroPosition + position;
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

    public int getDeltaPositionWithoutReset() {
        int curEncoderPosition = getEncoderPosition();
        int deltaPosition = curEncoderPosition - lastEncoderReading;
        return deltaPosition;
    }
}
