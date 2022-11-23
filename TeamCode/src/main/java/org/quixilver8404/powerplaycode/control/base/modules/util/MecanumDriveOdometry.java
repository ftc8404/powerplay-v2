package org.quixilver8404.powerplaycode.control.base.modules.util;

import android.util.Log;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;
import org.quixilver8404.powerplaycode.util.Vector3;

import java.util.Arrays;

public class MecanumDriveOdometry {

    final Vector3[] setup;
    protected Vector3 pos;
    protected Vector3 vel;

    protected Vector3 d_pos;

    public MecanumDriveOdometry(final Vector3[] setup, final Vector3 pos) {
        this.setup = setup;
        this.pos = pos;
        vel = new Vector3();

        d_pos = new Vector3();
    }

    public double[][] makeA() {
        final double alpha = pos.theta();
        final double[][] A = new double[][]{
                {Math.cos(alpha + setup[0].theta()), Math.sin(alpha + setup[0].theta()), Math.hypot(setup[0].x(), setup[0].y()) * Math.sin(setup[0].theta() - Math.atan2(setup[0].y(), setup[0].x()))},
                {Math.cos(alpha + setup[1].theta()), Math.sin(alpha + setup[1].theta()), Math.hypot(setup[1].x(), setup[1].y()) * Math.sin(setup[1].theta() - Math.atan2(setup[1].y(), setup[1].x()))},
                {Math.cos(alpha + setup[2].theta()), Math.sin(alpha + setup[2].theta()), Math.hypot(setup[2].x(), setup[2].y()) * Math.sin(setup[2].theta() - Math.atan2(setup[2].y(), setup[2].x()))},
                {Math.cos(alpha + setup[3].theta()), Math.sin(alpha + setup[3].theta()), Math.hypot(setup[3].x(), setup[3].y()) * Math.sin(setup[3].theta() - Math.atan2(setup[3].y(), setup[3].x()))}
        };
        return A;
    }

    public void update(final double[] deltaCount, final double dt) {
        final OLSMultipleLinearRegression TheRegressor = new OLSMultipleLinearRegression();
        final double[][] A = makeA();
        TheRegressor.newSampleData(deltaCount, A);
        TheRegressor.setNoIntercept(true);
        TheRegressor.newSampleData(deltaCount, A);
        final double[] x = TheRegressor.estimateRegressionParameters();
        Log.d("d_pos", Arrays.toString(x));
        d_pos = new Vector3(-x[0], -x[1], -x[2]);
        pos = pos.addVector(d_pos);
        vel = Vector3.ScalarMultiply(d_pos, 1/dt);
        Log.v("odometry info", "deltaCount=" + Arrays.toString(deltaCount) + "  d_pos=" + d_pos.toString() + "  pos=" + pos.toString());
    }

    public Vector3 getPosition() {
        return pos;
    }

    public Vector3 getVelocity() {
        return vel;
    }

    public Vector3 getDeltaPos() {
        return d_pos;
    }

    public void setPosition(final Vector3 pos) {
        this.pos = pos;
    }
}