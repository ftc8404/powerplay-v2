package org.quixilver8404.powerplay.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Represents a motor that never interacts with encoders
 */
public class EncoderlessMotor extends PowerMotor {

    public EncoderlessMotor(String deviceName, DcMotorSimple.Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior, HardwareMap hwMap) {
        super(deviceName, direction, zeroPowerBehavior, hwMap);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public EncoderlessMotor(String deviceName, DcMotorSimple.Direction direction, HardwareMap hwMap) {
        this(deviceName, direction, DcMotor.ZeroPowerBehavior.BRAKE, hwMap);
    }
}
