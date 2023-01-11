package org.quixilver8404.skystone.control;

import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.NelderMeadSimplex;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.SimplexOptimizer;

import java.util.Arrays;

public class MSonicModule {
    public double[] calcPos(double x, double y, double phi, final double ideal, final double sens_x, final double sens_y, final double sens_j) {
        int equation = initial(x, y, phi, ideal, sens_x, sens_y, sens_j);
        System.out.println(equation);
        double[] pos = nelder(equation, x, y, phi, ideal, sens_x, sens_y, sens_j);
        return pos;
    }

    public static int initial(double x, double y, double phi, final double ideal, final double sens_x, final double sens_y, final double sens_j) {
//    double PosSX=r * Math.cos(theta + phi)+x;
//    double PosSY=r * Math.sin(theta + phi)+y;
//    double XendLasor= PosSX +\
        double field1 = 141;
        double lenght1 = (field1 - x - sens_x * Math.cos(phi) - sens_y * Math.sin(phi)) / (Math.cos(phi + sens_j));
        double field2 = 0;
        double lenght2 = (field2 - x - sens_x * Math.cos(phi) - sens_y * Math.sin(phi)) / (Math.cos(phi + sens_j));
        double lenght3 = (field1 - y - sens_x * Math.sin(phi) - sens_y * Math.cos(phi)) / (Math.sin(phi + sens_j));
        double lenght4 = (field2 - y - sens_x * Math.sin(phi) - sens_y * Math.cos(phi)) / (Math.sin(phi + sens_j));
        double finallenght = Math.min(Math.min(Math.pow(lenght1 - ideal, 2), Math.pow(lenght2 - ideal, 2)), Math.min(Math.pow(lenght3 - ideal, 2), Math.pow(lenght4 - ideal, 2)));
//        System.out.println("Final Lenght: " + finallenght);
        if (finallenght == Math.pow(lenght1 - ideal, 2)) {
            return 1;
        } else if (finallenght == Math.pow(lenght2 - ideal, 2)) {
            return 2;
        } else if (finallenght == Math.pow(lenght3 - ideal, 2)) {
            return 3;
        } else if (finallenght == Math.pow(lenght4 - ideal, 2)) {
            return 4;
        } else {
            return 0;
        }
    }

    public static double[] nelder(int equation, double x, double y, double phi, final double ideal, final double sens_x, final double sens_y, final double sens_j) {
        SimplexOptimizer optimizer = new SimplexOptimizer(1e-10, 1e-30);
        MultivariateFunction selectedEquation;
        if (equation == 1) {
            selectedEquation = new EquationOne(ideal,sens_x,sens_y,sens_j);
            final PointValuePair optimum =
                    optimizer.optimize(
                            new MaxEval(10000),
                            new ObjectiveFunction(selectedEquation),
                            GoalType.MINIMIZE,
                            new InitialGuess(new double[]{x, phi}),
                            new NelderMeadSimplex(new double[]{1, 1}));

//            System.out.println(Arrays.toString(optimum.getPoint()) + " : "
//                    + optimum.getValue());
            return optimum.getPoint();
        } else if (equation == 2) {
            selectedEquation = new EquationTwo(ideal, sens_x, sens_y, sens_j);
            final PointValuePair optimum =
                    optimizer.optimize(
                            new MaxEval(10000),
                            new ObjectiveFunction(selectedEquation),
                            GoalType.MINIMIZE,
                            new InitialGuess(new double[]{x, phi}),
                            new NelderMeadSimplex(new double[]{1, 1}));

//            System.out.println(Arrays.toString(optimum.getPoint()) + " : "
//                    + optimum.getValue());
            return optimum.getPoint();
        } else if (equation == 3) {
            selectedEquation = new EquationThree(ideal, sens_x, sens_y, sens_j);
            final PointValuePair optimum =
                    optimizer.optimize(
                            new MaxEval(10000),
                            new ObjectiveFunction(selectedEquation),
                            GoalType.MINIMIZE,
                            new InitialGuess(new double[]{y, phi}),
                            new NelderMeadSimplex(new double[]{1, 1}));

//            System.out.println(Arrays.toString(optimum.getPoint()) + " : "
//                    + optimum.getValue());
            return optimum.getPoint();
        } else if (equation == 4) {
            selectedEquation = new EquationFour(ideal, sens_x, sens_y, sens_j);
            final PointValuePair optimum =
                    optimizer.optimize(
                            new MaxEval(10000),
                            new ObjectiveFunction(selectedEquation),
                            GoalType.MINIMIZE,
                            new InitialGuess(new double[]{y, phi}),
                            new NelderMeadSimplex(new double[]{1, 1}));

//            System.out.println(Arrays.toString(optimum.getPoint()) + " : "
//                    + optimum.getValue());
            return optimum.getPoint();
        }
        return null;
    }

    private static class EquationOne implements MultivariateFunction {
        double ideal;
        double sens_x;
        double sens_y;
        double sens_j;
        public EquationOne(final double ideal, final double sens_x, final double sens_y, final double sens_j){
            this.ideal = ideal;
            this.sens_x = sens_x;
            this.sens_y = sens_y;
            this.sens_j = sens_j;
        }
        public double value(double[] variables) {
            final double x = variables[0];
            final double phi = variables[1];
            return Math.pow((((141 - x - sens_x * Math.cos(phi) - sens_y * Math.sin(phi)) / (Math.cos(phi + sens_j))-ideal)),2);
        }
    }
    private static class EquationTwo implements MultivariateFunction {
        double ideal;
        double sens_x;
        double sens_y;
        double sens_j;
        public EquationTwo(final double ideal, final double sens_x, final double sens_y, final double sens_j){
            this.ideal = ideal;
            this.sens_x = sens_x;
            this.sens_y = sens_y;
            this.sens_j = sens_j;
        }
        public double value(double[] variables) {
            final double x = variables[0];
            final double phi = variables[1];
            return Math.pow((((0 - x - sens_x * Math.cos(phi) - sens_y * Math.sin(phi)) / (Math.cos(phi + sens_j))-ideal)),2);
        }
    }
    private static class EquationThree implements MultivariateFunction {
        double ideal;
        double sens_x;
        double sens_y;
        double sens_j;
        public EquationThree(final double ideal, final double sens_x, final double sens_y, final double sens_j){
            this.ideal = ideal;
            this.sens_x = sens_x;
            this.sens_y = sens_y;
            this.sens_j = sens_j;
        }
        public double value(double[] variables) {
            final double y = variables[0];
            final double phi = variables[1];
            return Math.pow((((141 - y - sens_x * Math.sin(phi) - sens_y * Math.cos(phi)) / (Math.sin(phi + sens_j)))-ideal),2);
        }
    }
    private static class EquationFour implements MultivariateFunction {
        double ideal;
        double sens_x;
        double sens_y;
        double sens_j;
        public EquationFour(final double ideal, final double sens_x, final double sens_y, final double sens_j){
            this.ideal = ideal;
            this.sens_x = sens_x;
            this.sens_y = sens_y;
            this.sens_j = sens_j;
        }
        public double value(double[] variables) {
            final double y = variables[0];
            final double phi = variables[1];
            return Math.pow((((0 - y - sens_x * Math.sin(phi) - sens_y * Math.cos(phi)) / (Math.sin(phi + sens_j)))-ideal),2);
        }
    }
}
