//package org.quixilver8404.energize.control.algorithms;
//
//import org.apache.commons.math3.analysis.MultivariateFunction;
//import org.apache.commons.math3.optim.PointValuePair;
//import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.NelderMeadSimplex;
//
//import java.util.*;
//
//public class Optimizer {
//
//    final MultivariateFunction func;
//    final NelderMeadSimplex nms;
//    final Comparator<PointValuePair> comparator;
//    final double relevantDelta;
//    protected double lastVal;
//    final int numNoChange;
//    protected int count;
//    protected boolean hasConverged;
//    final int maxIterations;
//    final int minIterations;
//    protected int iterations;
//
//    public Optimizer(final MultivariateFunction func, final double[] startPos, final boolean maximize, final double relevantDelta, final int numNoChange, final int maxIterations, final int minIterations) {
//        this.func = func;
//        nms = new NelderMeadSimplex(startPos.length);
//        nms.build(startPos);
//        final int mul = maximize ? 1 : -1;
//        comparator = new Comparator<PointValuePair>() {
//            @Override
//            public int compare(final PointValuePair p1, final PointValuePair p2) {
//                if (p1.getValue() > p2.getValue()) {
//                    return -1 * mul;
//                } else if (p1.getValue() < p2.getValue()) {
//                    return 1 * mul;
//                }  else {
//                    return 0;
//                }
//            }
//        };
//        this.relevantDelta = relevantDelta;
//        lastVal = Double.NaN;
//        this.numNoChange = numNoChange;
//        count = 0;
//        hasConverged = false;
//        this.maxIterations = maxIterations;
//        this.minIterations = minIterations;
//        iterations = 0;
//    }
//
//    public PointValuePair getBestPoint() { // TODO Fix
//        return Collections.min(new ArrayList<PointValuePair>(Arrays.asList(nms.getPoints())), comparator);
//    }
//
//    public PointValuePair iterateAndReturn() {
//
//        if (hasConverged) return getBestPoint();
//        nms.iterate(func, comparator);
//        final PointValuePair pvp = getBestPoint();
//        if (Double.isNaN(relevantDelta)) {
//            lastVal = pvp.getValue();
//        } else if (pvp.getValue() - lastVal < relevantDelta) {
//            lastVal = pvp.getValue();
//            count++;
//        }
//        if (count >= numNoChange && iterations < minIterations) {
//            hasConverged = true;
//        }
//        iterations++;
//        if (iterations >= maxIterations) {
//            hasConverged = true;
//        }
//        return pvp;
//    }
//
//    public boolean hasConverged() {
//        return hasConverged;
//    }
//
//    public int getIterations() {
//        return iterations;
//    }
//}