package org.quixilver8404.powerplay.control;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.quixilver8404.powerplay.util.Vector3;

import java.util.Arrays;

public class Odometry {

    protected final Vector3[] setup;
    protected Vector3 vel;
    protected Vector3 d_pos_raw;
    protected Vector3 d_pos;

    static final double dist_calibrationRad1 = 1 * (3381/3600d) * (87/90d);
    static final double dist_calibrationRad2 = 1 * (3381/3600d) * (87/90d);
    static final double dist_calibrationRad3 = 1 * (3381/3600d) * (87/90d);
    static final double dist_calibrationTan1 = 1;
    static final double dist_calibrationTan2 = 1;
    static final double dist_calibrationTan3 = 1;
//    protected OdometryEMA ema;
//
//    protected double ODO_RB;
//    protected double ODO_RA;

    protected Vector3 pos;

    public Odometry(final Vector3[] setup, final Vector3 startPos) {
        this.setup = setup;
        d_pos_raw = new Vector3();
        d_pos = new Vector3();
//        ema = new OdometryEMA(new Vector3(0.02, 0.02, 0.02), 70);

//        ODO_RB = setup[1].y();
//        ODO_RA = setup[0].x();

        pos = startPos;
    }

    protected double[][] makeA(final double alpha) {
        final double[][] A = new double[][]{
                {Math.cos(alpha + setup[0].theta()), Math.sin(alpha + setup[0].theta()), dist_calibrationRad1* Math.hypot(setup[0].x(), setup[0].y()) * Math.sin(setup[0].theta() - dist_calibrationTan1*(Math.atan2(setup[0].y(), setup[0].x())))},
                {Math.cos(alpha + setup[1].theta()), Math.sin(alpha + setup[1].theta()), dist_calibrationRad2* Math.hypot(setup[1].x(), setup[1].y()) * Math.sin(setup[1].theta() - dist_calibrationTan2*(Math.atan2(setup[1].y(), setup[1].x())))},
                {Math.cos(alpha + setup[2].theta()), Math.sin(alpha + setup[2].theta()), dist_calibrationRad3* Math.hypot(setup[2].x(), setup[2].y()) * Math.sin(setup[2].theta() - dist_calibrationTan3*(Math.atan2(setup[2].y(), setup[2].x())))}
        };
        return A;
    }

//    protected double[][] makeA2(final double prevAlpha, final double dAlpha) {
//        final double[][] A = new double[][] {
//                {(2*Math.cos(setup[0].theta()+prevAlpha+(dAlpha/2))*Math.sin(dAlpha/2))/dAlpha, (2*Math.sin(setup[0].theta()+prevAlpha+(dAlpha/2))*Math.sin(dAlpha/2)/dAlpha), Math.hypot(setup[0].x(), setup[0].y()) * Math.sin(setup[0].theta() - Math.atan2(setup[0].y(), setup[0].x()))},
//                {(2*Math.cos(setup[1].theta()+prevAlpha+(dAlpha/2))*Math.sin(dAlpha/2))/dAlpha, (2*Math.sin(setup[1].theta()+prevAlpha+(dAlpha/2))*Math.sin(dAlpha/2)/dAlpha), Math.hypot(setup[1].x(), setup[1].y()) * Math.sin(setup[1].theta() - Math.atan2(setup[1].y(), setup[1].x()))},
//                {(2*Math.cos(setup[2].theta()+prevAlpha+(dAlpha/2))*Math.sin(dAlpha/2))/dAlpha, (2*Math.sin(setup[2].theta()+prevAlpha+(dAlpha/2))*Math.sin(dAlpha/2)/dAlpha), Math.hypot(setup[2].x(), setup[2].y()) * Math.sin(setup[2].theta() - Math.atan2(setup[2].y(), setup[2].x()))}
//        };
//        return A;
//    }

    protected double[] solve(final RealMatrix matrixA, final double[] b) {
        final ArrayRealVector vectorb = new ArrayRealVector(b);
        final LUDecomposition luDecomposition = new LUDecomposition(matrixA);
        final DecompositionSolver decompositionSolver = luDecomposition.getSolver();
        if (decompositionSolver.isNonSingular()) {
            final RealVector solved = decompositionSolver.solve(vectorb);
            return solved.toArray();
        } else {
            return new double[]{0,0,0};
        }
    }
//    public void update(final double[] ds, double v, final double dtime) {
//
//        // calculate the change of [x, y, theta]
//        final double dt = (ds[2]-ds[1])/(2*ODO_RB);
//        final double dy = ds[0] - ODO_RA * dt;
//        final double dx = (ds[2]+ds[1])/2;
//
//        // translate the coordinate to the field's
//        final double ang = pos.theta() + dt/2;
//        d_pos.setX(  dx * Math.cos(ang) - dy * Math.sin(ang) );
//        d_pos.setY(  dx * Math.sin(ang) + dy * Math.cos(ang) );
//        d_pos.setTheta( dt );
//
//        vel = Vector3.ScalarMultiply(d_pos, 1/dtime);
//
//        pos.addVector(d_pos);
//    }

    public void update(final double[] deltaCount, final double dt, final double alpha) {

        System.out.println("Beginning odometry update");
        System.out.println("odo-dt: " + dt);
        System.out.println("deltaCount: " + Arrays.toString(deltaCount));
        System.out.println("alpha: " + alpha);

        final double[][] A = makeA(alpha);
        final RealMatrix matrixA = MatrixUtils.createRealMatrix(A);
        final double[] x = solve(matrixA, deltaCount);
//        final double dAlpha = x[2];

//        final double[][] A2;
//        if (Math.abs(dAlpha) < 1e-10d ) { //TODO: Figure out if this is actually necessary. I sincerely doubt it
//            A2 = makeA(alpha + dAlpha/2);
//        } else {
//            A2 = makeA2(alpha, dAlpha);
//        }
//        final RealMatrix matrixA2 = MatrixUtils.createRealMatrix(A2);
//        final double[] x2 = solve(matrixA2, deltaCount);

//        System.out.println("x2: " + Arrays.toString(x2));

        pos = Vector3.AddVector(pos, new Vector3(x[0], x[1], x[2]));
//        pos = new Vector3(x[0], x[1], x[2]);

        d_pos.setX(x[0]/* *ema.getMhat().x()*/);
        d_pos.setY(x[1]/* *ema.getMhat().y()*/);
        d_pos.setTheta(x[2]/* *ema.getMhat().theta()*/);

        d_pos_raw.setX(x[0]);
        d_pos_raw.setY(x[1]);
        d_pos_raw.setTheta(x[2]);

        System.out.println("Delta pos: " + d_pos);

        vel = Vector3.ScalarMultiply(d_pos, 1/dt);
    }

    public Vector3 getVelocity() {
        return vel;
    }

    public Vector3 getRawDeltaPos() {
        return d_pos_raw;
    }

    public Vector3 getDeltaPos() {
        return d_pos;
    }

    public Vector3 getPosition() {
        return pos;
    }

//    public void updateEMA(final Vector3 d_abs, final Vector3 d_odo) {
//        ema.update(d_abs, d_odo);
//    }

    public void setPosition(final Vector3 pos) {
        this.pos = pos;
    }
}