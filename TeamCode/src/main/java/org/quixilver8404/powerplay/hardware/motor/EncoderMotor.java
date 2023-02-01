package org.quixilver8404.powerplay.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.quixilver8404.powerplay.hardware.sensor.Encoder;

/**
 * Represents a motor that can move at a power and can track its encoder count
 * and will reset the motor's encoder
 */
public class EncoderMotor extends PowerMotor {

    protected final Encoder encoder;

    EncoderMotor(String deviceName, DcMotorSimple.Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior, DcMotor.RunMode mode, HardwareMap hwMap) {
        super(deviceName, direction, zeroPowerBehavior, hwMap);
        motor.setMode(mode);
        encoder = new Encoder(this);
    }

    public EncoderMotor(String deviceName, DcMotorSimple.Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior, HardwareMap hwMap) {
        this(deviceName, direction, zeroPowerBehavior, DcMotor.RunMode.RUN_WITHOUT_ENCODER, hwMap);
    }

    public EncoderMotor(String deviceName, DcMotorSimple.Direction direction, HardwareMap hwMap) {
        this(deviceName, direction, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, hwMap);
    }

    public Encoder getEncoder() {
        return encoder;
    }
}
