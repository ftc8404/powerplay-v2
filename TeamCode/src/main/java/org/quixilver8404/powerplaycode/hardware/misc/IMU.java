package org.quixilver8404.powerplaycode.hardware.misc;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.quixilver8404.powerplaycode.hardware.HardwareDevice;
import org.quixilver8404.powerplaycode.util.Angles;
import org.quixilver8404.powerplaycode.util.Vector3;

public class IMU extends HardwareDevice {

    protected final BNO055IMU imu;
    protected Orientation orientation;
    protected Position position;
    protected Velocity velocity;
    protected Acceleration acceleration;
    protected double heading;
    protected double deltaHeading;

    public IMU(final HardwareMap hwMap, final String deviceName, final BNO055IMU.AngleUnit angleUnit,final BNO055IMU.AccelUnit accelUnit, final String calibrationFileName) {
        super(deviceName);
        imu = hwMap.get(BNO055IMU.class, "imu");
        final BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = angleUnit;
        imuParams.accelUnit = accelUnit;
        imuParams.calibrationDataFile = calibrationFileName;
        imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(imuParams);
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        heading = 0;
        deltaHeading = 0;
        update();
    }

    public IMU(final HardwareMap hwMap, final String deviceName, final BNO055IMU.AngleUnit angleUnit,final BNO055IMU.AccelUnit accelUnit) {
        super(deviceName);
        imu = hwMap.get(BNO055IMU.class, "imu");
        final BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = angleUnit;
        imuParams.accelUnit = accelUnit;
        imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(imuParams);
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        heading = 0;
        deltaHeading = 0;
        update();
    }

    public IMU(final HardwareMap hwMap, final String deviceName, final BNO055IMU.Parameters params) {
        this(hwMap, deviceName, params.angleUnit, params.accelUnit, params.calibrationDataFile);
    }

    public void setIMUParameters(final BNO055IMU.AngleUnit newAngleUnit,final BNO055IMU.AccelUnit newAccelUnit, final String newCalibrationFileName) {
        final BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = newAngleUnit;
        imuParams.accelUnit = newAccelUnit;
        imuParams.calibrationDataFile = newCalibrationFileName;
        imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(imuParams);
    }

    public BNO055IMU.Parameters getParameters() {
        return imu.getParameters();
    }

    public void stopAccelerationIntegration() {
        imu.stopAccelerationIntegration();
    }

    public void startAccelerationIntegration(final Position initialPosition, final Velocity initialVelocity, final int msPollInterval) {
        imu.startAccelerationIntegration(initialPosition, initialVelocity, msPollInterval);
    }

    public Orientation getAngularOrientation() {
        return orientation;
    }

    public Position getPosition() {
        return position;
    }

    public Vector3 getRobotPosition() {
        return new Vector3(position.x, position.y, getAngularOrientation().thirdAngle);
    }

    public Vector3 getRobotVelocity() {
        final Velocity vel = getVelocity();
        return new Vector3(vel.xVeloc, vel.yVeloc, imu.getAngularVelocity().toAngleUnit(AngleUnit.RADIANS).zRotationRate);
    }

    public Velocity getVelocity() {
        return velocity;
    }

    public Acceleration getAcceleration() {
        return acceleration;
    }

    public void update() {
        final Orientation newOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        deltaHeading = Angles.zeroToPi(newOrientation.secondAngle - orientation.secondAngle);
        orientation = newOrientation;
        heading += deltaHeading;
        position = imu.getPosition().toUnit(DistanceUnit.METER);
        velocity = imu.getVelocity().toUnit(DistanceUnit.METER);
        acceleration = imu.getAcceleration().toUnit(DistanceUnit.METER);
    }

    public double getHeading() {
        return heading;
    }

    public double getDeltaHeading() {
        return deltaHeading;
    }

    public void setHeading(final double heading) {
        this.heading = heading;
    }
}

