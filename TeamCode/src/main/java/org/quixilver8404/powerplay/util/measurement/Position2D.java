package org.quixilver8404.powerplay.util.measurement;

/**
 * Represents a 2D position without direction.
 */
public class Position2D {

    public final Distance x;
    public final Distance y;

    public Position2D(final Distance x, final Distance y) {
        this.x = x;
        this.y = y;
    }

    public static Position2D addPositions(final Position2D p1, final Position2D p2) {
        return new Position2D(Distance.addDistances(p1.x, p2.x),
                Distance.addDistances(p1.y, p2.y));
    }

    public static Position2D subtractPositions(final Position2D p1, final Position2D p2) {
        return new Position2D(Distance.subtractDistances(p1.x, p2.x),
                Distance.subtractDistances(p1.y, p2.y));
    }
}
