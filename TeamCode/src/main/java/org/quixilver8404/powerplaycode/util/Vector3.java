package org.quixilver8404.powerplaycode.util;

import org.apache.commons.math3.linear.ArrayRealVector;
import java.util.Locale;

public class Vector3 {

    protected double x;
    protected double y;
    protected double theta;

//    public Vector3(final org.quixilver8404.breakout.util.Vector3 v) {
//        x = v.x;
//        y = v.y;
//        theta = v.theta;
//    }

    public Vector3() {
        this.x = 0;
        this.y = 0;
        this.theta = 0;
    }

    /**
     * Initializes x, y, and theta
     * @param x - change in x
     * @param y - change in y
     * @param theta - change in theta
     */
    public Vector3(final double x, final double y, final double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public double theta() {
        return theta;
    }

    public void setX(final double x) {
        this.x = x;
    }

    public void setY(final double y) {
        this.y = y;
    }

    public void setTheta(final double theta) {
        this.theta = theta;
    }

    public Vector3 addVector(final Vector3 v) {
        return new Vector3(x+v.x, y+v.y, theta+v.theta);
    }

    public Vector3 scalarMultiply(final double a) {
        return new Vector3(x*a, y*a, theta*a);
    }

    public Vector3 mul(final Vector3 v) {
        return new Vector3(v.x() * x, v.y() * y, v.theta() * theta);
    }

    public Vector3 div(final Vector3 v) {
        return new Vector3(x/v.x(), y/v.y(), theta/v.theta());
    }

    public Vector3 sqrt() {
        return new Vector3(Math.sqrt(x), Math.sqrt(y), Math.sqrt(theta));
    }

    public Vector3 square() {
        return mul(this);
    }

    /**
     * Adds two vectors
     * @param v1 - Vector One
     * @param v2 - Vector Two
     * @return - Sum of the vectors
     */
    public static Vector3 AddVectorStandard(final Vector3 v1, final Vector3 v2) {
        return new Vector3(v1.x+v2.x, v1.y+v2.y, Angles.zeroToPi(v1.theta+v2.theta));
    }

    public static Vector3 AddVector(final Vector3 v1, final Vector3 v2) {
        return new Vector3(v1.x+v2.x, v1.y+v2.y, v1.theta+v2.theta);
    }

    /**
     * Subtracts two vectors
     * @param v1 - Vector One
     * @param v2 - Vector Two
     * @return - Difference of the vectors
     */
    public static Vector3 SubtractVectorStandard(final Vector3 v1, final Vector3 v2) {
        return new Vector3(v1.x-v2.x, v1.y-v2.y, Angles.zeroToPi(v1.theta-v2.theta));
    }

    public static Vector3 SubtractVector(final Vector3 v1, final Vector3 v2) {
        return new Vector3(v1.x-v2.x, v1.y-v2.y, v1.theta-v2.theta);
    }

    public static Vector3 ScalarMultiply(final Vector3 v1, final double a) {
        return new Vector3(v1.x() * a, v1.y()*a, v1.theta*a);
    }

    /**
     * Saves x, y, and theta and returns them as a string
     * @return - Array in string form with x, y, and theta
     */
    public String toString() {
        return String.format(Locale.ENGLISH, "(%.3f, %.3f, %.3f°)", x, y, theta*180/Math.PI);
    }

    /**
     * Makes a new double Array with x, y, and theta
     * @return - double array with x, y, and theta
     */
    public double[] toArray() {
        return new double[] {x,y,theta};
    }

    /**
     * Makes a new vector
     * @return - new method that clones toArray()
     */
    public ArrayRealVector toRealVector() {
        return new ArrayRealVector(toArray());
    }

    public org.quixilver8404.breakout.util.Vector3 toBreakoutVec() {
        return new org.quixilver8404.breakout.util.Vector3(x, y, theta);
    }

    public Vector3 copy() {
        return new Vector3(x, y, theta);
    }

    public Vector3 abs() {
        return new Vector3(Math.abs(x), Math.abs(y), Math.abs(theta));
    }
}
