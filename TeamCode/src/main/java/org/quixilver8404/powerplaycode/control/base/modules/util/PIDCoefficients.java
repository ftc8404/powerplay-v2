package org.quixilver8404.powerplaycode.control.base.modules.util;

public class PIDCoefficients {
    public final double kP;
    public final double kI;
    public final double kD;

    PIDCoefficients(final double kP, final double kI, final double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
