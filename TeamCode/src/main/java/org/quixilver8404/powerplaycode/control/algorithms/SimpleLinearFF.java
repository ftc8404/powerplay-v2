package org.quixilver8404.powerplaycode.control.algorithms;

/**
 * Goes from 0 to pos while staying withing the specified acceleration and deceleration. If the set velocity is possible within these constraints, it will "cruise" at that velocity.
 * https://www.desmos.com/calculator/qx7lts0gmf
 */
public class SimpleLinearFF {

    public final double x0;
    public final double delta_x;
    public final double a;
    public final double d;
    public final double v;

    public final double s;
    public final double t1;
    public final double V;
    public final double t3;
    public final double t2;
    public final boolean middle_segment;
    public final double c1;
    public final double c2;
    public final double V1;
    public final double t4;
    public final double t5;

    public final double final_time;

    public SimpleLinearFF(final double start_pos, final double end_pos, final double acceleration, final double deceleration, final double speed) {
        x0 = start_pos;
        delta_x = end_pos - start_pos;
        a = acceleration;
        d = deceleration;
        v = speed;

        s = Math.signum(delta_x);
        t1 = v/a;
        V = Math.max(v, a*t1);
        t3 = V/d;
        t2 = (s*delta_x - (1d/2)*(a)*Math.pow(t1, 2) + (1d/2)*(d)*Math.pow(t3, 2) - V*t3)/V;
        middle_segment = t2 > 0;

        V1 = Math.sqrt(s*2*a*d*delta_x/(d+a));
        t4 = V1/a;
        t5 = V1/d;

        if (middle_segment) {
            c1 = s*(1d/2)*a*Math.pow(t1, 2);
            c2 = - s*V*t1 - s*(d/2)*Math.pow(t1+t2, 2) + s*(V + d*(t1 + t2));
            final_time = t3;
        } else {
            c1 = - s*V1*t4 + s*(1d/2)*a*Math.pow(t4, 2) - s*d*(1d/2)*Math.pow(t4, 2);
            c2 = 0;
            final_time = t5;
        }
    }

    public double getPos(final double t) {
        if (middle_segment) {
            if (t < 0) {
                return x0;
            } else if (0 <= t && t <= t1) {
                return s*(1d/2)*a*Math.pow(t, 2) + x0;
            } else if (t1 <= t && t <= t1 + t2) {
                return c1 + s*V*(t - t1) + x0;
            } else if (t <= t1 + t2 + t3) { //t >= t1 + t2
                return c1 + c2 - s*(1d/2)*d*Math.pow(t, 2) + x0;
            } else {
                return x0 + delta_x;
            }
        } else {
            if (t < 0) {
                return x0;
            } else if (0 <= t && t <= t4) {
                return s*(1d/2)*a*Math.pow(t, 2) + x0;
            } else if (t4 <= t && t <= t4 + t5) {
                return Math.pow(t, 2)*(-s*(1d/2)*d) + t*(s*d*t4+s*V1) + c1 + x0;
            } else {
                return x0 + delta_x;
            }
        }
    }

}
