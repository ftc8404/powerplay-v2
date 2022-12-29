package org.quixilver8404.powerplaycode.hardware.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.quixilver8404.powerplaycode.hardware.HardwareDevice;

public class BaseMotor extends HardwareDevice {

    protected final DcMotorEx motor;
    protected boolean isPowered;
    protected double prevPower;
    protected double maxPower;

    public BaseMotor(final String deviceName, final DcMotorSimple.Direction direction, final DcMotor.ZeroPowerBehavior zeroPowerBehavior, final HardwareMap hwMap) {
        super(deviceName);
        motor = (DcMotorEx) hwMap.dcMotor.get(deviceName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMotorEnable();
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);

        isPowered = true;
        prevPower = 0;
        maxPower = 1;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public BaseMotor(String deviceName, DcMotorSimple.Direction direction, HardwareMap hwMap) {
        this(deviceName, direction, DcMotor.ZeroPowerBehavior.BRAKE, hwMap);
    }

    public void disable() {
        if (isPowered) {
            motor.setMotorDisable();
            isPowered = false;
        }
    }

    public void enable() {
        if (!isPowered) {
            motor.setMotorEnable();
            isPowered = true;
        }
    }

    public DcMotorEx getInternalMotor() {
        return motor;
    }

    public boolean isPowered() {
        return isPowered;
    }

    public void setPower(final double power) {
        final double cappedPower = Math.min(Math.max(power, -maxPower), maxPower);

        if (Math.abs(cappedPower  - prevPower) >= 1e-2) {
            prevPower = cappedPower;
            motor.setPower(cappedPower);
        }
    }

    public void setMaxPower(final double maxPower) {
        this.maxPower = maxPower;
    }

    public double getPower() {
        return motor.getPower();
    }

    public void setMode(final DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public double getPrevPower() {
        return prevPower;
    }
}
