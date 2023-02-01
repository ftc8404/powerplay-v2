package org.quixilver8404.powerplay.control;

import java.util.ArrayList;

public class PPPathBuilder {

    public static final int CURVATURE_SAMPLE_SIZE = 2;
    public static final int TANGENT_HEADING_SAMPLE_SIZE = 1;
    public static final double INJECT_SPACING_INCHES = 1.0;
    public static final double MIN_POWER = 0.2;

    /**
     * Constructs a curve using a few vertices. The heading will set as tangent to the curve.
     */
    public static PurePursuitPath buildPathFromVertices(double[] xVertices, double[] yVertices, double maxPower, boolean stopAtEnd) {
        double[][] injectedPath = injectPoints(xVertices, yVertices, INJECT_SPACING_INCHES);
        double[] xVals = injectedPath[0];
        double[] yVals = injectedPath[1];

        double b = 0.144;

        double[] headings = tangentHeadings(xVals, yVals, TANGENT_HEADING_SAMPLE_SIZE);
        double[] powers = computeOptimalPowers(xVals, yVals, maxPower, stopAtEnd);

        return new WaypointPPPath(xVals, yVals, headings, powers);
    }

    /**
     * Constructs a smooth curve using a few vertices and headings
     */
    public static PurePursuitPath buildPathFromVertices(double[] xVals, double[] yVals, double[] headings, double maxPower, boolean stopAtEnd) {
        double[] powers = computeOptimalPowers(xVals, yVals, maxPower, stopAtEnd);
        return new WaypointPPPath(xVals, yVals, headings, powers);
    }

    /**
     * Returns a new array of equal length to xVals and yVals corresponding to the distance each
     * point is from the start of the path.
     */
    public static double[] distAlongPaths(double[] xVals, double[] yVals) {
        int n = xVals.length;
        if (yVals.length != n) {
            throw new RuntimeException("Array lengths do no match");
        }
        double[] distAlongPaths = new double[n];
        double curLength = 0;
        for (int i = 1; i < n; i++) {
            double segmentLength = Math.hypot(xVals[i] - xVals[i - 1], yVals[i] - yVals[i - 1]);
            curLength += segmentLength;
            distAlongPaths[i] = curLength;
        }
        return distAlongPaths;
    }

    /**
     * Returns a new array of equal length to xVals and yVals corresponding to the curvature at
     * each point.
     * Sample size refers to the number points before and after to use for calculating curvature
     * and should be at least 1.
     * Points within sampleSize from the start and end will have a curvature of 0.
     */
    public static double[] curvatures(double[] xVals, double[] yVals, int sampleSize) {
        int n = xVals.length;
        if (yVals.length != n) {
            throw new RuntimeException("Array lengths do no match");
        }
        double[] curvatureVals = new double[n];
        for (int i = sampleSize; i < n - sampleSize; i++) {
            double x1 = xVals[i];
            double y1 = yVals[i];
            double x2 = xVals[i - sampleSize];
            double y2 = yVals[i - sampleSize];
            double x3 = xVals[i + sampleSize];
            double y3 = yVals[i + sampleSize];

            if (x1 == x2) {
                x1 += 0.0001;
            }
            double k1 = 0.5 * (x1 * x1 + y1 * y1 - x2 * x2 - y2 * y2) / (x1 - x2);
            double k2 = (y1 - y2) / (x1 - x2);
            double b = 0.5 * (x2 * x2 - 2 * x2 * k1 + y2 * y2 - x3 * x3 + 2 * x3 * k1 - y3 * y3) / (x3 * k2 - y3 + y2 - x2 * k2);
            double a = k1 - k2 * b;
            double r = Math.sqrt((x1 - a) * (x1 - a) + (y1 - b) * (y1 - b));
            double curvature = 1 / r;
            if (Double.isNaN(curvature)) {
                curvature = 0;
            }
            curvatureVals[i] = curvature;
        }
        return curvatureVals;
    }

    /**
     * Computes the powers based on the path curvature and also limits the acceleration.
     */
    public static double[] computeOptimalPowers(double[] xVals, double[] yVals, double maxPower, boolean stopAtEnd) {
        int n = xVals.length;
        if (yVals.length != n) {
            throw new RuntimeException("Array lengths do no match");
        }
        double[] curvatures = curvatures(xVals, yVals, CURVATURE_SAMPLE_SIZE);
        double[] powers = new double[n];
        for (int i = 0; i < n; i++) {
            double power = PathFollowModule.POWER_PER_INCH_RADIUS / curvatures[i];
            if (power > maxPower) {
                power = maxPower;
            } else if (power < MIN_POWER) {
                power = MIN_POWER;
            }
            powers[i] = power;
        }
        double[] distAlongPaths = distAlongPaths(xVals, yVals);
        if (stopAtEnd) {
            powers[powers.length - 1] = 0;
        }
        limitAcceleration(powers, distAlongPaths);
        return powers;
    }

    /**
     * Returns a new array of equal length to xVals and yVals corresponding to the tangent at
     * each point.
     * Sample size refers to the number points before and after to use for calculating the angle of
     * the secant line and should be at least 1.
     */
    public static double[] tangentHeadings(double[] xVals, double[] yVals, int sampleSize) {
        int n = xVals.length;
        if (yVals.length != n) {
            throw new RuntimeException("Array lengths do no match");
        }
        double[] headings = new double[n];
        for (int i = 0; i < n; i++) {
            int index1 = Math.max(i - sampleSize, 0);
            int index2 = Math.min(i + sampleSize, n - 1);
            headings[i] = Math.atan2(yVals[index2] - yVals[index1], xVals[index2] - xVals[index1]);
        }
        return headings;
    }

    /**
     * Limits the acceleration and deceleration, preferring to only decrease velocities.
     * This will also enforce the minimum velocity. The new powers will be written into powerVals.
     */
    public static void limitAcceleration(double[] powerVals, double[] distAlongPaths) {
        int n = powerVals.length;
        if (distAlongPaths.length != n) {
            throw new RuntimeException("Array lengths do no match");
        }
        for (int i = 0; i < n; i++) {
            if (powerVals[i] < MIN_POWER) {
                powerVals[i] = MIN_POWER;
            }
        }
        for (int i = 1; i < n; i++) {
            double segLength = distAlongPaths[i] - distAlongPaths[i - 1];
            double prevPower = powerVals[i - 1];
            double curMaxPower = Math.sqrt(prevPower * prevPower + 2 * segLength * PathFollowModule.MAX_ACCELERATION_POWER_PER_INCH);
            powerVals[i] = Math.min(powerVals[i], curMaxPower);
        }
        for (int i = n - 2; i >= 0; i--) {
            double segLength = distAlongPaths[i + 1] - distAlongPaths[i];
            double prevPower = powerVals[i + 1];
            double curMaxPower = Math.sqrt(prevPower * prevPower + 2 * segLength * PathFollowModule.MAX_DECELERATION_POWER_PER_INCH);
            powerVals[i] = Math.min(powerVals[i], curMaxPower);
        }
    }

    /**
     * Adds points onto a path with the given spacing. The returned array has length 2, where the
     * first element is an array of x values and the second element is an array of y values.
     */
    public static double[][] injectPoints(double[] xVals, double[] yVals, double spacing) {
        int n = xVals.length;
        if (yVals.length != n) {
            throw new RuntimeException("Array lengths do no match");
        }
        ArrayList<Double> moreXVals = new ArrayList<>();
        ArrayList<Double> moreYVals = new ArrayList<>();
        for (int i = 0; i < n - 1; i++) {
            double x1 = xVals[i];
            double y1 = yVals[i];
            double x2 = xVals[i + 1];
            double y2 = yVals[i + 1];
            double dx = x2 - x1;
            double dy = y2 - y1;
            double segLength = Math.hypot(x2 - x1, y2 - y1);
            int numPoints = (int) Math.round(segLength / spacing);
            for (int j = 0; j < numPoints; j++) {
                double frac = (double) j / (double) numPoints;
                moreXVals.add(x1 + dx * frac);
                moreYVals.add(y1 + dy * frac);
            }
        }
        moreXVals.add(xVals[n - 1]);
        moreYVals.add(yVals[n - 1]);
        double[][] newPath = new double[2][moreXVals.size()];
        for (int i = 0; i < moreXVals.size(); i++) {
            newPath[0][i] = moreXVals.get(i);
            newPath[1][i] = moreYVals.get(i);
        }
        return newPath;
    }
}