package org.quixilver8404.skystone.control;

import org.quixilver8404.skystone.util.Tunable;
import org.quixilver8404.skystone.util.measurement.Distance;

public class SlidesModule {

    public enum SlidePositionPreset {
        GROUND(0),
        ABOVE_DRIVE(450),
        JUNC_1(1000),
        JUNC_2(2070),
        JUNC_3(2300),
        JUNC_4(6000);

        public final double HEIGHT_INCHES;

        SlidePositionPreset(final double height) {
            this.HEIGHT_INCHES = height;
        }
    }

    // should be slightly lower than ABOVE_DRIVE height
    @Tunable
    public final static double CLEAR_DRIVE_HEIGHT = 350;

    @Tunable
    public final static Distance START_POSITION = Distance.ZERO;

    @Tunable
    public static final double WINCH_DIAMETER_INCH = 60.0 / 25.4; // TODO tune
    @Tunable
    public static final int LIFT_MOTOR_COUNTS_PER_REV = 560;

    @Tunable
    public static final double STALL_POWER = 0.09; // TODO tune
    @Tunable
    public static final double MAX_UPWARD_POWER_INCREMENT = 1.0 - STALL_POWER;
    @Tunable
    public static final double MAX_DOWNWARD_POWER_INCREMENT = 1.0; // TODO tune
    @Tunable
    public static final double DOWNWARD_POWER_IN_DEAD_ZONE = 0.45; // TODO tune
    @Tunable
    public static final int MAX_POWER_DOWN_IN_DEAD_ZONE_MILLIS = 3000;

    @Tunable
    public static final double MAX_HEIGHT_INCHES = 44.0; // TODO tune
    // the zone where the lift should not actively hold itself up or go down at a power
    @Tunable
    public static final double BOTTOM_DEAD_ZONE_INCHES = 1.0;  // TODO tune
    // the amount of wiggle room with the string
    @Tunable
    public static final double SLACK_INCHES = 0.12; // TODO tune

    // accounts for a 90:60 gear ratio
    private final double countsPerInch = (double) LIFT_MOTOR_COUNTS_PER_REV / (Math.PI * WINCH_DIAMETER_INCH) * (90.0 / 60.0);

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

    private boolean atMaxHeight = false;
    private boolean atMinHeight = false;

    public SlidesModule() {
        targetPower = 0;
        targetPosition = START_POSITION;
        curPosition = targetPosition;
        rawEncoderReading = 0;
        bottomEncoderReading = (int) (-curPosition.getValue(Distance.Unit.INCHES) * countsPerInch);
        zeroPowerStartTimeMillis = -1000000;
    }

    public synchronized void update(HardwareCollection hwCollection) {
        // TODO add logic using susanModule.isSafetoLowerSlides
        // - if taking manual power input, ignore downward power
        // that would smash slides into base
        // - if using PID to move to set position, ignore target
        // positions that are lower than SlidePositionPreset.ABOVE_DRIVE

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

        if (runAtPower) {
            if (curPosition.getValue(Distance.Unit.INCHES) >= MAX_HEIGHT_INCHES && targetPower >= 0 || atMaxHeight) {
                atMaxHeight = true;
                moveToPosition(new Distance(MAX_HEIGHT_INCHES, Distance.Unit.INCHES), hwCollection);
            } else if (curPosition.getValue(Distance.Unit.INCHES) <= 0 && targetPower <= 0 || atMinHeight) {
                atMinHeight = true;
                moveToPosition(Distance.ZERO, hwCollection);
            } else {
                runAtPower(targetPower, hwCollection);
            }
        } else {
            moveToPosition(targetPosition, hwCollection);
        }

    }

    /**
     * Adjusts the reference encoder position for when the lift is at the bottom so the lift
     * now reads that it is at the bottom.
     */
    public synchronized void setCurPositionToBottom() {
        bottomEncoderReading = rawEncoderReading;
    }

    /**
     * Runs the lift to reach a specified position
     */
    private void moveToPosition(Distance position, HardwareCollection hwCollection) {
        if (powerDownInDeadZoneStartTimeMillis == -1) {
            powerDownInDeadZoneStartTimeMillis = hwCollection.clock.getRunningTimeMillis();
        }
        Distance newTargetPos = position;
        if (newTargetPos.getValue(Distance.Unit.METERS) < 0) {
            newTargetPos = Distance.ZERO;
        } else if (newTargetPos.getValue(Distance.Unit.INCHES) >= MAX_HEIGHT_INCHES || atMaxHeight) {
            atMaxHeight = true;
            newTargetPos = new Distance(MAX_HEIGHT_INCHES, Distance.Unit.INCHES);
        } else if (newTargetPos.getValue(Distance.Unit.INCHES) <= 0 || atMinHeight) {
            atMinHeight = true;
            newTargetPos = new Distance(0, Distance.Unit.INCHES);
        }

        double newPosInches = newTargetPos.getValue(Distance.Unit.INCHES);
        double curPosInches = curPosition.getValue(Distance.Unit.INCHES);

        if (newPosInches < BOTTOM_DEAD_ZONE_INCHES && curPosInches < BOTTOM_DEAD_ZONE_INCHES) {
            if (hwCollection.clock.getRunningTimeMillis() - powerDownInDeadZoneStartTimeMillis < MAX_POWER_DOWN_IN_DEAD_ZONE_MILLIS) {
                setRawPower(-DOWNWARD_POWER_IN_DEAD_ZONE, hwCollection);
                setCurPositionToBottom();
            } else {
                setRawPower(0, hwCollection);
            }
            positionPID.reset();
        } else {
            powerDownInDeadZoneStartTimeMillis = hwCollection.clock.getRunningTimeMillis();
            Distance offset = Distance.subtractDistances(curPosition, newTargetPos);
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
     * Only the left lift motor's encoder is used
     */
    private Distance readCurPosition(HardwareCollection hwCollection) {
        rawEncoderReading = hwCollection.slidesMotor1.getEncoder().getEncoderPosition();
        double inches = (rawEncoderReading - bottomEncoderReading) / countsPerInch;
        inches -= SLACK_INCHES;
        if (inches < 0) {
            inches = 0;
        }
        return new Distance(inches, Distance.Unit.INCHES);
    }

    public synchronized boolean areSlidesLowered() {
        return curPosition.getValue(Distance.Unit.INCHES) > CLEAR_DRIVE_HEIGHT;
    }

    /**
     * Moves the lift to the target position. This will set the velocity to 0.
     * This will zero out the target power so only targetPosition controls the lift.
     */
    public synchronized void setTargetPosition(Distance targetPosition) {
        this.targetPosition = targetPosition;
        targetPower = 0;
        runAtPower = false;
        if (targetPosition.getValue(Distance.Unit.INCHES) < MAX_HEIGHT_INCHES) {
            atMaxHeight = false;
        }
        if (targetPosition.getValue(Distance.Unit.INCHES) > 0) {
            atMinHeight = false;
        }
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
        if (targetPower < 0) {
            atMaxHeight = false;
        }
        if (targetPower > 0) {
            atMinHeight = false;
        }
    }
}

