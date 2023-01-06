package org.quixilver8404.skystone.util.measurement;

public class AngularVelocity {
    public static final AngularVelocity ZERO = new AngularVelocity(0, Unit.RADIANS_PER_SEC);

    /**
     * Valid angular velocity units
     */
    public enum Unit {
        RADIANS_PER_SEC(1), DEGREES_PER_SEC(Math.PI / 180d),
        REVOLUTIONS_PER_SEC(Math.PI * 2);
        private final double toRadiansPerSec;

        Unit(double toRadiansPerSec) {
            this.toRadiansPerSec = toRadiansPerSec;
        }
    }

    private final double radiansPerSec;

    /**
     * Creates a new angular velocity representing the input
     */
    public AngularVelocity(double angularVelocity, Unit unit) {
        radiansPerSec = angularVelocity * unit.toRadiansPerSec;
    }

    /**
     * Gets the angular velocity stored in this object
     */
    public double getValue(Unit unit) {
        return radiansPerSec / unit.toRadiansPerSec;
    }

    /**
     * Returns an angular velocity representing addend1 - addend2.
     */
    public static AngularVelocity addAngularVelocities(AngularVelocity addend1, AngularVelocity addend2) {
        return new AngularVelocity(addend1.radiansPerSec + addend2.radiansPerSec, Unit.RADIANS_PER_SEC);
    }

    /**
     * Returns an angular velocity representing minuend - subtrahend.
     */
    public static AngularVelocity subtractAngularVelocities(AngularVelocity minuend, AngularVelocity subtrahend) {
        return new AngularVelocity(minuend.radiansPerSec - subtrahend.radiansPerSec, Unit.RADIANS_PER_SEC);
    }
}
