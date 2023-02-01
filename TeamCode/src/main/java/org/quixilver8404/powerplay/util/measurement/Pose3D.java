package org.quixilver8404.powerplay.util.measurement;

/**
 * Represents a 3D position with a direction in 2d space.
 * x and y represent horizontal translation. z represents vertical translation.
 * The direction is determined by first applying the heading in the plane containing x and y.
 * Then, the direction is pivoted upward/downward depending on pitch.
 */
public class Pose3D extends Position3D {

    public final Angle heading;
    public final Angle pitch;

    /**
     * x and y represent horizontal translation. z represents vertical translation.
     * The direction is determined by first applying the heading in the plane containing x and y.
     * Then, the direction is pivoted upward/downward depending on pitch.
     */
    public Pose3D(Distance x, Distance y, Distance z, Angle heading) {
        super(x, y, z);
        this.heading = heading;
        pitch = new Angle(0, Angle.Unit.RADIANS);
    }

    public Pose3D() {
        super(new Distance(0, Distance.Unit.METERS), new Distance(0, Distance.Unit.METERS), new Distance(0, Distance.Unit.METERS));
        heading = new Angle(0, Angle.Unit.RADIANS);
        pitch = new Angle(0, Angle.Unit.RADIANS);
    }
}
