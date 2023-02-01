package org.quixilver8404.powerplay.util.measurement;

/**
 * Represents a 2D position with a direction.
 */
public class Pose2D extends Position2D {

    public static final Pose2D ZERO = new Pose2D(Distance.ZERO, Distance.ZERO, Angle.ZERO);

    public final Angle heading;

    public Pose2D(Distance x, Distance y, Angle heading) {
        super(x, y);
        this.heading = heading;
    }
}
