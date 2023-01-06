package org.quixilver8404.skystone.control;

public class WaypointPPPath implements PurePursuitPath {

    private final int n;
    private final int upper;
    private final double[] xVals;
    private final double[] yVals;
    private final double[] headingVals;
    private final double[] powerVals;
    private final double[] distAlongPaths;

    private final double length;

    /**
     * Curvature sample size refers to the number points before and after to use for calculating curvature and should be at least 1.
     */
    public WaypointPPPath(double[] xVals, double[] yVals, double[] headingVals, double[] powerVals) {
        n = xVals.length;
        if (yVals.length != n || headingVals.length != n || powerVals.length != n) {
            throw new RuntimeException("Array lengths do no match");
        }
        upper = n - 1;
        this.xVals = xVals;
        this.yVals = yVals;
        this.headingVals = headingVals;
        this.powerVals = powerVals;

        distAlongPaths = PPPathBuilder.distAlongPaths(xVals, yVals);
        length = distAlongPaths[upper];
    }

    @Override
    public double x(double t) {
        return interpolate(xVals, t);
    }

    @Override
    public double y(double t) {
        return interpolate(yVals, t);
    }

    @Override
    public double heading(double t) {
        return interpolate(headingVals, t);
    }

    @Override
    public double power(double t) {
        return interpolate(powerVals, t);
    }

    @Override
    public double distAlongPaths(double t) {
        return interpolate(distAlongPaths, t);
    }

    @Override
    public double atDist(double dist) {
        if (dist <= 0) {
            return 0;
        } else if (dist >= length) {
            return upper;
        }

        // binary search
        int low = 0;
        int high = upper;
        while (true) {
            int mid = (low + high) / 2;
            if (distAlongPaths[mid] <= dist) {
                low = mid;
            } else {
                high = mid;
            }
            if (low >= high) {
                break;
            } else if (low + 1 == high) {
                if (distAlongPaths[high] == dist) {
                    low = high;
                }
                break;
            }
        }
        double t = low + (dist - distAlongPaths[low]) / (distAlongPaths[low + 1] - distAlongPaths[low]);
        if (t < 0) {
            return 0;
        } else if (t > upper) {
            return upper;
        } else {
            return t;
        }
    }

    @Override
    public double length() {
        return length;
    }


    @Override
    public double closest(double x, double y, double start, double end) {
        if (start < 0) {
            start = 0;
        }
        if (end > upper) {
            end = upper;
        }
        int startIndex = (int) start;
        int endIndex = (int) Math.ceil(end);
        if (startIndex >= endIndex || start >= end) {
            return start;
        }

        double minDistSquared = Double.MAX_VALUE;
        double bestT = start;

        for (int i = startIndex; i < endIndex; i++) {
            double x1 = xVals[i];
            double y1 = yVals[i];
            double x2 = xVals[i + 1];
            double y2 = yVals[i + 1];
            double a = x2 - x1;
            double b = y2 - y1;
            double h = x1 - x;
            double k = y1 - y;
            double tSegment = -(a * h + b * k) / (a * a + b * b);

            if (tSegment > 1) {
                tSegment = 1;
            } else if (tSegment < 0) {
                tSegment = 0;
            }

            double t = i + tSegment;
            if (t < start) {
                t = start;
            } else if (t > end) {
                t = end;
            }

            double curDistSquared = getSquaredDist(x, y, x(t), y(t));
            if (curDistSquared < minDistSquared) {
                minDistSquared = curDistSquared;
                bestT = t;
            }
        }

        return bestT;
    }

    @Override
    public double lookahead(double x, double y, double dist, double start, double end) {
        if (start < 0) {
            start = 0;
        }
        if (end > upper) {
            end = upper;
        }
        int startIndex = (int) start;
        int endIndex = (int) Math.ceil(end);
        if (startIndex >= endIndex || start >= end) {
            return start;
        }

        double distSquared = dist * dist;

        for (int i = startIndex; i < endIndex; i++) {
            // for each segment, check to see if it intersects the lookahead circle
            double eX = xVals[i];
            double eY = yVals[i];
            double lX = xVals[i + 1];
            double lY = yVals[i + 1];
            double dX = lX - eX;
            double dY = lY - eY;
            double fX = eX - x;
            double fY = eY - y;
            double a = dX * dX + dY * dY;
            double b = 2 * (fX * dX + fY * dY);
            double c = (fX * fX + fY * fY) - distSquared;
            double discriminant = b * b - 4 * a * c;
            if (discriminant >= 0) {
                // possible intersection
                discriminant = Math.sqrt(discriminant);

                // second intersection point, where the segment "exits" the circle
                double t2 = (-b + discriminant) / (2 * a);

                if (t2 >= 0 && t2 <= 1) {
                    double t = i + t2;
                    if (t >= start && t <= end) {
                        return t;
                    }
                }
            }
        }

        if (getSquaredDist(x, y, x(end), y(end)) <= distSquared) {
            return end;
        }

        return closest(x, y, start, end);
    }

    @Override
    public double upper() {
        return upper;
    }

    @Override
    public double distFromEnd(double x, double y) {
        return Math.hypot(xVals[upper] - x, yVals[upper] - y);
    }

    private double interpolate(double[] vals, double t) {
        if (t <= 0) {
            return vals[0];
        }
        if (t >= vals.length - 1) {
            return vals[vals.length - 1];
        }

        int index = (int) t;
        double partial = t - index;

        return vals[index] + partial * (vals[index + 1] - vals[index]);
    }

    private double getSquaredDist(double deltaX, double deltaY) {
        return deltaX * deltaX + deltaY * deltaY;
    }

    private double getSquaredDist(double x1, double y1, double x2, double y2) {
        return getSquaredDist(x2 - x1, y2 - y1);
    }
}
