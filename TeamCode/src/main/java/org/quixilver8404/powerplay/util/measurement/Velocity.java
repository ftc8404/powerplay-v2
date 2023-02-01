package org.quixilver8404.powerplay.util.measurement;

public class Velocity {
    public static final Velocity ZERO = new Velocity(0, Unit.INCHES_PER_SEC);

    public enum Unit {
        METERS_PER_SEC(1), CENTIMETERS_PER_SEC(0.01),
        INCHES_PER_SEC(0.0254), FEET_PER_SEC(0.3048),
        MM_PER_SEC(0.001);

        private final double toMetersPerSec;

        Unit(double toMetersPerSec) {
            this.toMetersPerSec = toMetersPerSec;
        }
    }

    private final double metersPerSec; // The value of this velocity, represented in meters

    /**
     * Creates a new velocity representing the input velocity assuming
     * that it is given in the specified unit.
     */
    public Velocity(double velocity, Unit unit) {
        metersPerSec = velocity * unit.toMetersPerSec;
    }

    /**
     * Gets the value of this velocity in the specified unit.
     */
    public double getValue(Unit unit) {
        return metersPerSec / unit.toMetersPerSec;
    }

    /**
     * Returns a Velocity with value of v1 + v2
     */
    public static Velocity addDistances(Velocity b1, Velocity b2) {
        return new Velocity(b1.metersPerSec + b2.metersPerSec, Unit.METERS_PER_SEC);
    }

    /**
     * Returns a Velocity with value of v1 - v2
     */
    public static Velocity subtractDistances(Velocity v1, Velocity v2) {
        return new Velocity(v1.metersPerSec - v2.metersPerSec, Unit.METERS_PER_SEC);
    }
}
