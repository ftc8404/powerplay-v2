package org.quixilver8404.skystone.util.measurement;

public class Angle {
    public static final double RADS_PER_REV = Math.PI * 2;
    public static final Angle ZERO = new Angle(0, Unit.RADIANS);

    /**
     * Valid angle units
     */
    public enum Unit {
        RADIANS(1), DEGREES(Math.PI / 180d);
        private final double toRadians;
        private final double perRev;

        Unit(double toRadians) {
            this.toRadians = toRadians;
            perRev = RADS_PER_REV / toRadians;
        }
    }

    private final double standardRads; // The value of this angle in radians, [0, 2*pi)

    /**
     * Creates a new angle representing the input angle assuming
     * that it is given in the specified units.
     */
    public Angle(double angle, Unit unit) {
        standardRads = toStandard(angle * unit.toRadians, Unit.RADIANS);
    }

    /**
     * Returns the numeric representation of this angle in the specified unit.
     * The returned value is always the smallest, non-negative value representing this angle.
     */
    public double getStandard(Unit unit) {
        return standardRads / unit.toRadians;
    }

    /**
     * Returns the smallest, non-negative value equivalent to the
     * given angle for the specified unit.
     */
    public static double toStandard(double angle, Unit unit) {
        double temp = angle % unit.perRev;
        if (temp < 0) {
            temp += unit.perRev;
        }
        return temp;
    }

    /**
     * Returns an angle representing addend1 - addend2.
     */
    public static Angle addAngles(Angle addend1, Angle addend2) {
        return new Angle(addend1.standardRads + addend2.standardRads, Unit.RADIANS);
    }

    /**
     * Returns an angle representing minuend - subtrahend.
     */
    public static Angle subtractAngles(Angle minuend, Angle subtrahend) {
        return new Angle(minuend.standardRads - subtrahend.standardRads, Unit.RADIANS);
    }
}
