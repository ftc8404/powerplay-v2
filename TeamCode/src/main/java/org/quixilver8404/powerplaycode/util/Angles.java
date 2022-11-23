package org.quixilver8404.powerplaycode.util;

public class Angles {
    //adjusts angles to the specified bounds

    public static double zeroToPi(final double angle) {
        final double temp = angle % (2*Math.PI);
        if (temp < 0) {
            return temp + 2*Math.PI;
        }
        return temp;
    }

    public static double negPi2posPi(final double angle) {
        // reduce the angle
        double temp =  angle % (2*Math.PI);

        // force it to be the positive remainder, so that 0 <= angle < 2PI
        temp = (temp + (2*Math.PI)) % (2*Math.PI);

        // force into the minimum absolute value residue class, so that -PI < angle <= PI
        if (temp > Math.PI) {
            temp -= 2*Math.PI;
        }

        return temp;
    }
}
