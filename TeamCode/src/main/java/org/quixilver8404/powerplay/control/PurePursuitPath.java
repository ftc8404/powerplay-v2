package org.quixilver8404.powerplay.control;

/**
 * Defines a parametric path with the parameter "t" ranging from 0 to some upper limit
 */
public interface PurePursuitPath {

    /**
     * Returns the x coordinate at "t".
     */
    double x(double t);

    /**
     * Returns the y coordinate at "t".
     */
    double y(double t);

    /**
     * Returns the heading at "t" in radians.
     */
    double heading(double t);

    /**
     * Returns the power at "t" in units per second.
     */
    double power(double t);

    /**
     * Returns the distance along the path from the start at "t".
     */
    double distAlongPaths(double t);

    /**
     * Returns the paramter "t" for the point at the specified distance from the start.
     */
    double atDist(double dist);

    /**
     * Returns the total length of the path.
     */
    double length();

    /**
     * Returns the parameter "t", searching between start and end inclusive.
     */
    double closest(double x, double y, double start, double end);

    /**
     * Returns the parameter "t", searching between start and end inclusive.
     */
    double lookahead(double x, double y, double dist, double start, double end);

    /**
     * Returns the max value of "t".
     */
    double upper();

    /**
     * Returns how far the specified point is from the last point on the path.
     */
    double distFromEnd(double x, double y);
}
