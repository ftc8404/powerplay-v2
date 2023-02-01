package org.quixilver8404.powerplay.util.measurement;

public class Distance {
    public static final Distance ZERO = new Distance(0, Unit.INCHES);

    public enum Unit {
        METERS(1d), CENTIMETERS(0.01), MILLIMETERS(0.001), INCHES(0.0254), FEET(0.3048);

        private final double toMeters;

        Unit(double toMeters) {
            this.toMeters = toMeters;
        }
    }

    private final double meters; // The value of this distance, represented in meters

    /**
     * Creates a new distance representing the input distance assuming
     * that it is given in the specified unit.
     */
    public Distance(double distance, Unit unit) {
        meters = distance * unit.toMeters;
    }

    /**
     * Gets the value of this distance in the specified unit.
     */
    public double getValue(Unit unit) {
        return meters / unit.toMeters;
    }

    /**
     * Returns a Distance with value of d1 + d2
     */
    public static Distance addDistances(Distance d1, Distance d2) {
        return new Distance(d1.meters + d2.meters, Unit.METERS);
    }

    /**
     * Returns a Distance with value of d1 - d2
     */
    public static Distance subtractDistances(Distance d1, Distance d2) {
        return new Distance(d1.meters - d2.meters, Unit.METERS);
    }
}
