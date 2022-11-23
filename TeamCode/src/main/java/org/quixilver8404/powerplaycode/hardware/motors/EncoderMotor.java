package org.quixilver8404.powerplaycode.hardware.motors;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.quixilver8404.powerplaycode.hardware.sensors.Encoder;

public class EncoderMotor extends BaseMotor{

    protected final Encoder encoder;

    EncoderMotor(final String deviceName, final DcMotorSimple.Direction direction, final DcMotor.ZeroPowerBehavior zeroPowerBehavior, final DcMotor.RunMode mode, final HardwareMap hwMap, final LynxModule revHub) {
        super(deviceName, direction, zeroPowerBehavior, hwMap);
        motor.setMode(mode);
        if (mode.isPIDMode()) { motor.setPower(1); }
        encoder = new Encoder(this, revHub);
    }

    public EncoderMotor(final String deviceName, final DcMotorSimple.Direction direction, final DcMotor.ZeroPowerBehavior zeroPowerBehavior, final DcMotor.RunMode mode, final Encoder encoder, final HardwareMap hwMap) {
        super(deviceName, direction, zeroPowerBehavior, hwMap);
        motor.setMode(mode);
        if (mode.isPIDMode()) { motor.setPower(1); }
        this.encoder = encoder;
    }

    public EncoderMotor(final String deviceName, final DcMotorSimple.Direction direction, final DcMotor.ZeroPowerBehavior zeroPowerBehavior, final HardwareMap hwMap, final LynxModule revHub) {
        this(deviceName, direction, zeroPowerBehavior, DcMotor.RunMode.RUN_WITHOUT_ENCODER, hwMap, revHub);
    }

    public EncoderMotor(final String deviceName, final DcMotorSimple.Direction direction, final HardwareMap hwMap, final LynxModule revHub) {
        this(deviceName, direction, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, hwMap, revHub);
    }

    public Encoder getEncoder() {
        return encoder;
    }

    public double getEncoderPosition() {
        return encoder.getEncoderPosition();
    }

    public double getEncoderVelocity() {
        return encoder.getEncoderVelocity();
    }

    public void setEncoderPosition(final int position) {
        encoder.setPosition(position);
    }
}
