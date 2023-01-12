package org.quixilver8404.skystone.control;

import android.util.Log;

import org.quixilver8404.skystone.util.Tunable;
import org.quixilver8404.skystone.util.measurement.Distance;

public class SlidesModule {

    public enum SlidePositionPreset {
        GROUND(0), // TODO tune
        ABOVE_DRIVE(12.0), // TODO tune
        JUNC_1(5.0), // TODO tune
        JUNC_2(10.0), // TODO tune
        JUNC_3(15.0), // TODO tune
        JUNC_4(20.0); // TODO tune

        public final double HEIGHT_INCHES;

        SlidePositionPreset(final double heightInches) {
            this.HEIGHT_INCHES = heightInches;
        }
    }

    @Tunable
    // buffer around min/max height constraint to avoid unstable oscillating behavior
    public static final double HEIGHT_CONSTRAINT_BUFFER_INCHES = 0.5; // TODO tune

    // should be slightly lower than ABOVE_DRIVE height
    @Tunable
    public final static double CLEAR_DRIVE_HEIGHT_INCHES = 10.0;

    @Tunable
    public static final double STALL_POWER = 0.09;

    public static final double MAX_UPWARD_POWER_INCREMENT = 1.0 - STALL_POWER;
    public static final double MAX_DOWNWARD_POWER_INCREMENT = 1.0 + STALL_POWER;

    @Tunable
    public static final double DOWNWARD_POWER_IN_DEAD_ZONE = 0.45;
    @Tunable
    public static final int MAX_POWER_DOWN_IN_DEAD_ZONE_MILLIS = 3000;
    @Tunable
    public static final double MAX_HEIGHT_INCHES = 37.0; // TODO tune
    // the zone where the lift should not actively hold itself up or go down at a power
    @Tunable
    public static final double BOTTOM_DEAD_ZONE_INCHES = 1.0;
    // the amount of wiggle room with the string
    @Tunable
    public static final double SLACK_INCHES = 0.12;

    @Tunable
    public static final double WINCH_DIAMETER_INCH = 60.0 / 25.4;
    @Tunable
    // bare motor counts/rev * gearbox reduction
    public static final int SLIDE_MOTOR_COUNTS_PER_REV = 28 * 25;

    public static final double COUNTS_PER_INCH = (double) SLIDE_MOTOR_COUNTS_PER_REV / (Math.PI * WINCH_DIAMETER_INCH);

    private long powerDownInDeadZoneStartTimeMillis = -1;

    private double targetPower;
    private Distance targetPosition;

    private boolean runAtPower = false;

    private Distance curPosition;
    private int rawEncoderReading;

    private int bottomEncoderReading; // reading when the lift is all the way down

    @Tunable
    private final PIDController positionPID = new PIDController(0.6, 0, 1200, 300); // TODO tune

    @Tunable
    public static final int PID_DELAY_MILLIS = 500; // the time before switching from power of zero to PID

    private int zeroPowerStartTimeMillis; // for tracking when to switch from power to PID

    public SlidesModule() {
        targetPower = 0;
        targetPosition = Distance.ZERO;
        curPosition = targetPosition;
        rawEncoderReading = 0;
        bottomEncoderReading = 0;
        zeroPowerStartTimeMillis = -1000000;
    }

    public synchronized void update(SusanModule susanModule, HardwareCollection hwCollection) {
        int currentTimeMillis = hwCollection.clock.getRunningTimeMillis();

        curPosition = readCurPosition(hwCollection);
        if (curPosition.getValue(Distance.Unit.METERS) < 0) {
            setCurPositionToBottom();
            curPosition = Distance.ZERO;
        }

        // automatically switch to PID mode after some time at zero power
        if (runAtPower) {
            if (targetPower != 0) {
                zeroPowerStartTimeMillis = currentTimeMillis;
            } else {
                if (currentTimeMillis - zeroPowerStartTimeMillis > PID_DELAY_MILLIS) {
                    setTargetPosition(curPosition);
                }
            }
        }

        double minHeightInches = susanModule.isSafeToLowerSlides()
                ? 0 : SlidePositionPreset.ABOVE_DRIVE.HEIGHT_INCHES;

        double curPosInches = curPosition.getValue(Distance.Unit.INCHES);
        
        if (runAtPower) {
            if (curPosInches > MAX_HEIGHT_INCHES && targetPower > 0) {
                runAtPower(0, hwCollection);
            } else if (curPosInches < minHeightInches && targetPower < 0) {
                runAtPower(0, hwCollection);
            } else {
                runAtPower(targetPower, hwCollection);
            }
        } else {
            if (targetPosition.getValue(Distance.Unit.INCHES) >= MAX_HEIGHT_INCHES - HEIGHT_CONSTRAINT_BUFFER_INCHES) {
                moveToPosition(new Distance(MAX_HEIGHT_INCHES, Distance.Unit.INCHES), hwCollection);
            } else if (targetPosition.getValue(Distance.Unit.INCHES) <= minHeightInches + HEIGHT_CONSTRAINT_BUFFER_INCHES) {
                moveToPosition(new Distance(minHeightInches, Distance.Unit.INCHES), hwCollection);
            } else {
                moveToPosition(targetPosition, hwCollection);
            }
        }

    }

    /**
     * Adjusts the reference encoder position for when the lift is at the bottom so the lift
     * now reads that it is at the bottom.
     */
    private synchronized void setCurPositionToBottom() {
        bottomEncoderReading = rawEncoderReading;
    }

    /**
     * Runs the lift to reach a specified position
     */
    private void moveToPosition(Distance targetPosition, HardwareCollection hwCollection) {
        if (powerDownInDeadZoneStartTimeMillis == -1) {
            powerDownInDeadZoneStartTimeMillis = hwCollection.clock.getRunningTimeMillis();
        }

        double targetPosInches = targetPosition.getValue(Distance.Unit.INCHES);
        double curPosInches = curPosition.getValue(Distance.Unit.INCHES);

        if (targetPosInches < BOTTOM_DEAD_ZONE_INCHES && curPosInches < BOTTOM_DEAD_ZONE_INCHES) {
            if (hwCollection.clock.getRunningTimeMillis() - powerDownInDeadZoneStartTimeMillis < MAX_POWER_DOWN_IN_DEAD_ZONE_MILLIS) {
                setRawPower(-DOWNWARD_POWER_IN_DEAD_ZONE, hwCollection);
                setCurPositionToBottom();
            } else {
                setRawPower(0, hwCollection);
            }
            positionPID.reset();
        } else {
            powerDownInDeadZoneStartTimeMillis = hwCollection.clock.getRunningTimeMillis();
            Distance offset = Distance.subtractDistances(curPosition, targetPosition);
            double offsetInches = offset.getValue(Distance.Unit.INCHES);
            double outputPower = positionPID.loop(offsetInches, hwCollection.clock.getRunningTimeMillis());
            if (Math.abs(outputPower) > 1) {
                positionPID.resetIntegral();
            }
            runAtPower(outputPower, hwCollection);
        }
    }

    /**
     * Runs the lift at a specified power in range [-1, 1], accounting for stall power
     */
    private void runAtPower(double power, HardwareCollection hwCollection) {
        double curPosInches = curPosition.getValue(Distance.Unit.INCHES);
        if (!(curPosInches < BOTTOM_DEAD_ZONE_INCHES && power == 0)) {
            powerDownInDeadZoneStartTimeMillis = hwCollection.clock.getRunningTimeMillis();
        }
        if (curPosInches < BOTTOM_DEAD_ZONE_INCHES && power <= 0) {
            setRawPower(-DOWNWARD_POWER_IN_DEAD_ZONE, hwCollection);
            setCurPositionToBottom();
            positionPID.reset();
        } else {
            double adjustedPower = STALL_POWER;
            if (power >= 0) {
                adjustedPower += power * MAX_UPWARD_POWER_INCREMENT;
            } else {
                adjustedPower += power * MAX_DOWNWARD_POWER_INCREMENT;
            }
            setRawPower(adjustedPower, hwCollection);
            positionPID.reset();
        }
    }

    /**
     * Sets the power to both lift motors simultaneously
     */
    private void setRawPower(double power, HardwareCollection hwCollection) {
        hwCollection.slidesMotor1.setPower(power);
        hwCollection.slidesMotor2.setPower(power);
    }

    /**
     * Only one motor's encoder is used
     */
    private Distance readCurPosition(HardwareCollection hwCollection) {
        rawEncoderReading = hwCollection.slidesMotor1.getEncoder().getEncoderPosition();
        double inches = (rawEncoderReading - bottomEncoderReading) / COUNTS_PER_INCH;
        inches -= SLACK_INCHES;
        if (inches < 0) {
            inches = 0;
        }
        return new Distance(inches, Distance.Unit.INCHES);
    }

    public synchronized boolean areSlidesLowered() {
        return curPosition.getValue(Distance.Unit.INCHES) < CLEAR_DRIVE_HEIGHT_INCHES;
    }

    /**
     * Moves the lift to the target position. This will set the velocity to 0.
     * This will zero out the target power so only targetPosition controls the lift.
     */
    public synchronized void setTargetPosition(Distance targetPosition) {
        this.targetPosition = targetPosition;
        targetPower = 0;
        runAtPower = false;
    }

    public synchronized void setTargetPositionPreset(SlidePositionPreset targetPositionPreset) {
        setTargetPosition(new Distance(targetPositionPreset.HEIGHT_INCHES, Distance.Unit.INCHES));
    }

    /**
     * This will pause the PID and run by power. After a certain period of time at zero power,
     * the lift will switch to the PID position mode.
     */
    public synchronized void setTargetPower(double targetPower) {
        this.targetPower = targetPower;
        if (targetPower == 0 && runAtPower) {
            targetPosition = curPosition;
        }
        if (targetPower != 0) {
            runAtPower = true;
        }
    }

    public synchronized Distance getCurPosition() {
        return curPosition;
    }
}

