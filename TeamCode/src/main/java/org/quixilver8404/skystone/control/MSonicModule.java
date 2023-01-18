package org.quixilver8404.skystone.control;

import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.exception.TooManyEvaluationsException;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.NelderMeadSimplex;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.SimplexOptimizer;

import java.util.Arrays;

public class MSonicModule {
    public static double[] tripleThreat(double x, double y, double phi, double dist1, double dist2, double dist3) {
        Sensor s1 = new Sensor(-7.5 + 2, 17.5 / 2 - 2.625, -Math.PI / 2);
        Sensor s2 = new Sensor(7.5 - 1.75, 17.5 / 2 - 2.75, 0);
        Sensor s3 = new Sensor(7.5 - 2.375, 17.5 / 2 - 3.25, Math.PI / 2);
        s1.setEquation(initial(x, y, phi, dist1, s1));
        s2.setEquation(initial(x, y, phi, dist2, s2));
        s3.setEquation(initial(x, y, phi, dist3, s3));
        System.out.println("s1 Equation: " + s1.getEquation());
        System.out.println("s2 Equation: " + s2.getEquation());
        System.out.println("s3 Equation: " + s3.getEquation());
        double[] result = combineNelder(s1, s2, s3, dist1, dist2, dist3, x, y, phi);
        result[2] = (result[2]*180/Math.PI)%360;
        return result;
    }

    public static int initial(double x, double y, double phi, final double ideal, Sensor s) {
        double length1 = Math.pow((((141 - x - s.getSens_x() * Math.cos(phi) + s.getSens_y() * Math.sin(phi)) / (Math.sin(phi + s.getSens_j())) - ideal)), 2);
        double length2 = Math.pow((((0 - x - s.getSens_x() * Math.cos(phi) + s.getSens_y() * Math.sin(phi)) / (Math.sin(phi + s.getSens_j())) - ideal)), 2);
        double length3 = Math.pow((((141 - y - s.getSens_x() * Math.sin(phi) - s.getSens_y() * Math.cos(phi)) / (Math.cos(phi + s.getSens_j()))) - ideal), 2);
        double length4 = Math.pow((((0 - y - s.getSens_x() * Math.sin(phi) - s.getSens_y() * Math.cos(phi)) / (Math.cos(phi + s.getSens_j()))) - ideal), 2);
        double finalLength = Math.min(Math.min(length1, length2), Math.min(length3, length4));
        if (finalLength > 50) {
            return 0;
        }
        if (finalLength == length1) {
            return 1;
        } else if (finalLength == length2) {
            return 2;
        } else if (finalLength == length3) {
            return 3;
        } else if (finalLength == length4) {
            return 4;
        } else {
            return 0;
        }
    }

    public static double[] combineNelder(Sensor s1, Sensor s2, Sensor s3, double dist1, double dist2, double dist3, double x, double y, double phi) {
        SimplexOptimizer optimizer = new SimplexOptimizer(1e-10, 1e-30);
        MultivariateFunction eq1 = new Equation(s1, dist1);
        MultivariateFunction eq2 = new Equation(s2, dist2);
        MultivariateFunction eq3 = new Equation(s3, dist3);
        MultivariateFunction combine = new Combine(eq1, eq2, eq3);
        try {
            final PointValuePair optimum = optimizer.optimize(
                    new MaxEval(20000),
                    new ObjectiveFunction(combine),
                    GoalType.MINIMIZE,
                    new InitialGuess(new double[]{x, y, phi}),
                    new NelderMeadSimplex(new double[]{1, 1, 1}));
            return optimum.getPoint();
        } catch(TooManyEvaluationsException e){
            return new double[3];
        }
    }

    private static class Combine implements MultivariateFunction {
        MultivariateFunction eq1;
        MultivariateFunction eq2;
        MultivariateFunction eq3;

        public Combine(MultivariateFunction eq1, MultivariateFunction eq2, MultivariateFunction eq3) {
            this.eq1 = eq1;
            this.eq2 = eq2;
            this.eq3 = eq3;
        }

        public double value(double[] variables) {
            return eq1.value(variables) + eq2.value(variables) + eq3.value(variables);
        }
    }

    private static class Equation implements MultivariateFunction {
        Sensor s;
        double dist;

        public Equation(Sensor s, final double dist) {
            this.s = s;
            this.dist = dist;
        }

        public double value(double[] variables) {
            final double x = variables[0];
            final double y = variables[1];
            final double phi = variables[2];
            if (s.getEquation() == 1) {
                return Math.pow((((141 - x - s.getSens_x() * Math.cos(phi) + s.getSens_y() * Math.sin(phi)) / (Math.sin(phi + s.getSens_j())) - dist)), 2);
            } else if (s.getEquation() == 2) {
                return Math.pow((((0 - x - s.getSens_x() * Math.cos(phi) + s.getSens_y() * Math.sin(phi)) / (Math.sin(phi + s.getSens_j())) - dist)), 2);
            } else if (s.getEquation() == 3) {
                return Math.pow((((141 - y - s.getSens_x() * Math.sin(phi) - s.getSens_y() * Math.cos(phi)) / (Math.cos(phi + s.getSens_j()))) - dist), 2);
            } else if (s.getEquation() == 4) {
                return Math.pow((((0 - y - s.getSens_x() * Math.sin(phi) - s.getSens_y() * Math.cos(phi)) / (Math.cos(phi + s.getSens_j()))) - dist), 2);
            } else {
                return 0;
            }
        }
    }

    private static class Sensor {
        double sens_x;
        double sens_y;
        double sens_j;
        int equation;

        public Sensor(final double sens_x, final double sens_y, final double sens_j) {
            this.sens_x = sens_x;
            this.sens_y = sens_y;
            this.sens_j = sens_j;
        }

        public double getSens_x() {
            return sens_x;
        }

        public double getSens_y() {
            return sens_y;
        }

        public double getSens_j() {
            return sens_j;
        }

        public void setEquation(int equation) {
            this.equation = equation;
        }

        public int getEquation() {
            return equation;
        }
    }
}
