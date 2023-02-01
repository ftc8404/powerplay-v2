package org.quixilver8404.powerplay.util.measurement;

/**
 * Represents a 3D position without direction.
 */
public class Position3D extends Position2D {

    public final Distance z;

    public Position3D(Distance x, Distance y, Distance z) {
        super(x, y);
        this.z = z;
    }

    public static Position3D addPositions(final Position3D p1, final Position3D p2) {
        return new Position3D(Distance.addDistances(p1.x, p2.x),
                Distance.addDistances(p1.y, p2.y), Distance.addDistances(p1.z, p2.z));
    }

    public static Position3D subtractPositions(final Position3D p1, final Position3D p2) {
        return new Position3D(Distance.subtractDistances(p1.x, p2.x),
                Distance.subtractDistances(p1.y, p2.y), Distance.subtractDistances(p1.z, p2.z));
    }
}
