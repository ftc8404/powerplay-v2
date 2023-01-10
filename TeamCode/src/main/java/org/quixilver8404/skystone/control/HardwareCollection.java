package org.quixilver8404.skystone.control;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.quixilver8404.skystone.hardware.misc.Clock;
import org.quixilver8404.skystone.hardware.motor.EncoderlessMotor;
import org.quixilver8404.skystone.hardware.sensor.DeltaEncoder;
import org.quixilver8404.skystone.util.Tunable;

/**
 * Stores every hardware device attached to the robot
 */
public class HardwareCollection {

    // clock
    public final Clock clock;

    // control and expansion hub
    LynxModule controlHub;
    LynxModule expansionHub;

    // drive motors
    public final EncoderlessMotor driveMotorFL;
    public final EncoderlessMotor driveMotorFR;
    public final EncoderlessMotor driveMotorBL;
    public final EncoderlessMotor driveMotorBR;

    // drive motor direction tunables
    @Tunable
    public static final DcMotorSimple.Direction DRIVE_MOTOR_FL_DIRECTION = DcMotorSimple.Direction.FORWARD;
    @Tunable
    public static final DcMotorSimple.Direction DRIVE_MOTOR_FR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    @Tunable
    public static final DcMotorSimple.Direction DRIVE_MOTOR_BL_DIRECTION = DcMotorSimple.Direction.FORWARD;
    @Tunable
    public static final DcMotorSimple.Direction DRIVE_MOTOR_BR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // drive encoders
    public final DeltaEncoder driveEncoderLeft;
    public final DeltaEncoder driveEncoderRight;
    public final DeltaEncoder driveEncoderCenter;

    // drive encoder tunables
    @Tunable
    public static final DcMotorSimple.Direction DRIVE_ENCODER_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    @Tunable
    public static final DcMotorSimple.Direction DRIVE_ENCODER_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    @Tunable
    public static final DcMotorSimple.Direction DRIVE_ENCODER_CENTER_DIRECTION = DcMotorSimple.Direction.REVERSE;

    /**
     * May block slightly as all hardware is initialized, servos may snap to their
     * default positions
     */
    public HardwareCollection(HardwareMap hwMap) {
        // utilities
        clock = new Clock("globalClock");

        // for bulk reading
        controlHub = hwMap.get(LynxModule.class, "Control Hub");
//        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        refreshControlHubBulkData();
        expansionHub = hwMap.get(LynxModule.class, "Expansion Hub");
//        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        refreshExpansionHubBulkData();

        // drive motors
        driveMotorFL = new EncoderlessMotor("driveMotorFL", DRIVE_MOTOR_FL_DIRECTION, hwMap);
        driveMotorFR = new EncoderlessMotor("driveMotorFR", DRIVE_MOTOR_FR_DIRECTION, hwMap);
        driveMotorBL = new EncoderlessMotor("driveMotorBL", DRIVE_MOTOR_BL_DIRECTION, hwMap);
        driveMotorBR = new EncoderlessMotor("driveMotorBR", DRIVE_MOTOR_BR_DIRECTION, hwMap);

        // odometry encoders
        driveEncoderLeft = new DeltaEncoder(driveMotorBR, DRIVE_ENCODER_LEFT_DIRECTION);
        driveEncoderRight = new DeltaEncoder(driveMotorBL, DRIVE_ENCODER_RIGHT_DIRECTION);
        driveEncoderCenter = new DeltaEncoder(driveMotorFL, DRIVE_ENCODER_CENTER_DIRECTION);
    }

    /**
     * Reads bulk data from expansion hub 1
     */
    public void refreshControlHubBulkData() {
//        controlHub.getBulkData();
    }

    /**
     * Reads bulk data from expansion hub 2
     */
    public void refreshExpansionHubBulkData() {
//        expansionHub.getBulkData();
    }
}
