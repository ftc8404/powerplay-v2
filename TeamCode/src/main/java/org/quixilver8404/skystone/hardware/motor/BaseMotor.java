package org.quixilver8404.skystone.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.quixilver8404.skystone.hardware.HardwareDevice;

/**
 * Wrapper for a motor
 */
public abstract class BaseMotor extends HardwareDevice {

    protected final DcMotor motor;

    BaseMotor(String deviceName, DcMotorSimple.Direction direction, HardwareMap hwMap) {
        super(deviceName);
        motor = hwMap.dcMotor.get(deviceName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(direction);
    }

    public DcMotor getInternalMotor() {
        return motor;
    }
}
