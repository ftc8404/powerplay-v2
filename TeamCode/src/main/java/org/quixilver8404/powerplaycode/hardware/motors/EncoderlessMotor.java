package org.quixilver8404.powerplaycode.hardware.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EncoderlessMotor extends BaseMotor{
    public EncoderlessMotor(final String deviceName, final DcMotorSimple.Direction direction, final DcMotor.ZeroPowerBehavior zeroPowerBehavior, final HardwareMap hwMap) {
        super(deviceName, direction, zeroPowerBehavior, hwMap);
    }

    public EncoderlessMotor(final String deviceName, final DcMotorSimple.Direction direction, final HardwareMap hwMap) {
        this(deviceName, direction, DcMotor.ZeroPowerBehavior.BRAKE, hwMap);
    }
}
