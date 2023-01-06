package org.quixilver8404.skystone.hardware.sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.quixilver8404.skystone.hardware.HardwareDevice;
import org.quixilver8404.skystone.util.measurement.Angle;

/**
 * Represents an inertial magnetic unit sensor, such as the one inside the Rev Expansion Hub
 */
public class IMU extends HardwareDevice {

    /**
     * Encapsulates a reading of an angle in 3d space
     */
    public class IMUGyroReading {
        public final Angle pitch;
        public final Angle roll;
        public final Angle heading;

        public IMUGyroReading(Angle pitch, Angle roll, Angle heading) {
            this.pitch = pitch;
            this.roll = roll;
            this.heading = heading;
        }
    }

    private final BNO055IMU imu;
    private final BNO055IMU.Parameters imuParams;
    private final AxesOrder axesOrder;

    // which index angle corresponds to which properties
    private final int headingAngleIndex;
    private final int pitchAngleIndex;
    private final int rollAngleIndex;

    /**
     * Remember to call init() to initialize before polling any readings.
     *
     * @param axesOrder axis 1 is pitch, axis 2 is roll, and axis 3 is heading
     */
    public IMU(String deviceName, String calibrationDataFile, AxesOrder axesOrder, int headingAngleIndex, int pitchAngleIndex, int rollAngleIndex, HardwareMap hwMap) {
        super(deviceName);
        imu = hwMap.get(BNO055IMU.class, deviceName);
        imuParams = new BNO055IMU.Parameters();
        if (calibrationDataFile != null) {
            imuParams.calibrationDataFile = calibrationDataFile;
        }
        this.axesOrder = axesOrder;
        this.headingAngleIndex = headingAngleIndex;
        this.pitchAngleIndex = pitchAngleIndex;
        this.rollAngleIndex = rollAngleIndex;
    }

    /**
     * Initializes the sensor and may block for some time
     */
    public void init() {
        imu.initialize(imuParams);
    }

    public IMUGyroReading getGyro() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.RADIANS);
        Angle[] angleList = {new Angle(orientation.firstAngle, Angle.Unit.RADIANS), new Angle(orientation.secondAngle, Angle.Unit.RADIANS),
                new Angle(orientation.thirdAngle, Angle.Unit.RADIANS)};
        return new IMUGyroReading(angleList[pitchAngleIndex], angleList[rollAngleIndex], angleList[headingAngleIndex]);
    }

    public Angle getGyroHeading() {
        return getGyro().heading;
    }
}
