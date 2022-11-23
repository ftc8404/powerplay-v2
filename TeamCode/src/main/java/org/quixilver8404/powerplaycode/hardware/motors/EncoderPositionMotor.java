package org.quixilver8404.powerplaycode.hardware.motors;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.quixilver8404.powerplaycode.control.algorithms.PIDController;
import org.quixilver8404.powerplaycode.hardware.sensors.Encoder;

/**
 * Represents a motor that supports moving to a position using an internal PID loop
 * and will reset the motor's encoder
 */
public class EncoderPositionMotor extends EncoderMotor {

    protected double prevRunPower;
    protected int prevTargetPosition;
    protected int targetPosition;
    protected int tolerance = 0;
    protected final PIDController pid;

    /**
     * Sets both velocity PIDF coefficient and position P coefficient if velocityPIDFCoefficients is not null
     */
    public EncoderPositionMotor(final String deviceName, final DcMotorSimple.Direction direction, final DcMotor.ZeroPowerBehavior zeroPowerBehavior, final double kP, final double kI, final double kD, final Encoder encoder, final int targetPosition, final HardwareMap hwMap) {
        super(deviceName, direction, zeroPowerBehavior, DcMotor.RunMode.RUN_WITHOUT_ENCODER, encoder, hwMap);
//        motor.setTargetPosition(0);
        prevRunPower = 0;
        prevTargetPosition = targetPosition;
        this.targetPosition = targetPosition;
        tolerance = 0;
        pid = new PIDController(kP, kI, kD);
    }

    public EncoderPositionMotor(final String deviceName, final DcMotorSimple.Direction direction, final DcMotor.ZeroPowerBehavior zeroPowerBehavior, final double kP, final double kI, final double kD, final LynxModule revHub, final int targetPosition, final HardwareMap hwMap) {
        this(deviceName, direction, zeroPowerBehavior, kP, kI, kD, new Encoder(deviceName, direction, revHub, hwMap), targetPosition, hwMap);
    }

    /**
     * The input targetPosition is relative to the zero position after any resets to the encoder
     */
    public void runToPosition(final int targetPosition) {
        if (targetPosition != prevTargetPosition) {
            prevTargetPosition = targetPosition;
            motor.setTargetPosition(encoder.convertPositionToRaw(targetPosition));
        }
    }

    /**
     * Calls reset() on the internal encoder.
     * Zeros out the encoder, also adjusting the effect of runToPosition
     */
    public void resetEncoder() {
        encoder.reset();
    }


    public void setPositionTolerance(final int encoderTicks) {
        tolerance = encoderTicks;
    }

    public void setPIDCoefficients(final double kP, final double kI, final double kD) {
        pid.setkP(kP);
        pid.setkI(kI);
        pid.setkD(kD);
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public double updatePID(final double error, final double dt) {
        return pid.update(error, dt);
    }

    public double updatePID(final double error, final double feedForward, final double dt) {
        return pid.update(error, feedForward, dt);
    }
    public void resetPID(){
        pid.reset();
    }
}
