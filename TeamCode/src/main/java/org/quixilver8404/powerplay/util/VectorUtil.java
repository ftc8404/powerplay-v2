package org.quixilver8404.powerplay.util;

public class VectorUtil {

    public static double[] toUnitVector2D(double i, double j) {
        double magnitude = Math.hypot(i, j);
        return new double[]{i / magnitude, j / magnitude};
    }

    public static double[] toUnitVector2D(double[] vector) {
        return toUnitVector2D(vector[0], vector[1]);
    }

}
