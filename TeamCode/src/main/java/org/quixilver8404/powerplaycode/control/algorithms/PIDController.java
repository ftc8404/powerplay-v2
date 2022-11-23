package org.quixilver8404.powerplaycode.control.algorithms;

public class PIDController {

    protected double kP;
    protected double kI;
    protected double kD;

    protected double last_error;
    protected double integral_error;

    public PIDController(final double kP, final double kI, final double kD) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;

        last_error = Double.NaN;
        integral_error = 0;
    }

    public double update(final double error, final double dt) {
        final double ddt_error;
        if (Double.isNaN(last_error)) {
            ddt_error = 0;
        } else {
            ddt_error = (error - last_error)/dt;
        }
//        Log.d("PIDController", "Last_error" + last_error);
        last_error = error;
        integral_error += error*dt;
//        Log.d("PIDController", "Integral_error" + integral_error);
//        Log.d("PIDController", "Current_error" + error);
//        Log.d("PIDController", "Derivative_error" + ddt_error);
//        Log.d("PIDController", "Total" + (kP*error + kI*integral_error + kD*ddt_error));
        return kP*error + kI*integral_error + kD*ddt_error;
    }

    public double update(final double error, final double feedForward, final double dt) {
        final double ddt_error;
        if (Double.isNaN(last_error)) {
            ddt_error = 0;
        } else {
            ddt_error = (error - last_error)/dt;
        }
//        Log.d("PIDController", "Last_error" + last_error);
        last_error = error;
        integral_error += error*dt;
//        Log.d("PIDController", "Integral_error" + integral_error);
//        Log.d("PIDController", "Current_error" + error);
//        Log.d("PIDController", "Derivative_error" + ddt_error);
//        Log.d("PIDController", "Total" + (kP*error + kI*integral_error + kD*ddt_error));
        return kP*error + kI*integral_error + kD*ddt_error + feedForward;
    }

    public void setCoefficients(final double kP, final double kI, final double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setkP(final double kP) {
        this.kP = kP;
    }

    public void setkI(final double kI) {
        this.kI = kI;
    }

    public void setkD(final double kD) {
        this.kD = kD;
    }

    public double kP() {
        return kP;
    }

    public double kI() {
        return kI;
    }

    public double kD() {
        return kD;
    }

    public void reset(){
        last_error = Double.NaN;
        integral_error = 0;
    }
}
