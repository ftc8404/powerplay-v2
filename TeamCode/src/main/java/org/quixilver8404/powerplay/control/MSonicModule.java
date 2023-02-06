package org.quixilver8404.powerplay.control;

import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.NelderMeadSimplex;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.SimplexOptimizer;
import org.quixilver8404.powerplay.util.Vector3;

import java.util.Arrays;

public class MSonicModule {
    BaseRobot robot;
    Vector3 pos;
    public MSonicModule(BaseRobot robot) {
        //double equation = initial(x, y, phi, ideal1, ideal2, ideal3);
        //System.out.println(equation);
//        nelder(x, y, phi, ideal1, ideal2, ideal3);
        this.robot = robot;
        pos = robot.poseModule.getPos();
    }

    public double getPosX() {
        return pos.x();
    }

    public double getPosY() {
        return pos.y();
    }

    public double getPosTheta() {
        return pos.theta();
    }

    public double tripleInitial(double x, double y, double phi, double ideal1, double ideal2, double ideal3) {
        double s1 = initial(x,y,phi,ideal1,0,0,0,'n' );//config
        double s2 = initial(x,y,phi,ideal2,0,0,0,'s');//config
        double s3 = initial(x,y,phi,ideal3,0,0,0,'e');//config
        return s1+s2+s3;
    }

    public double initial(double x, double y, double phi, final double ideal, final double sens_x, final double sens_y, final double sens_j,final char sens_D_nsew) {
        double r     = Math.sqrt(sens_x * sens_x + sens_y * sens_y);
        //System.out.println("r: " + r );
        double theta;
        if (sens_x == 0) {theta = (Math.signum(sens_y) * Math.PI/2 + 2 * Math.PI)%Math.PI;}
        else {theta = Math.atan(sens_y/sens_x);}
        // System.out.println("theta: " + theta/Math.PI * 180 );
        if (theta<0){
            if (sens_y>0)   theta = Math.PI + theta;
            else theta = theta + 2* Math.PI;
        }

        //System.out.println("theta: " + theta/Math.PI * 180 );
        // System.out.println("sens_x = " + r*Math.cos(theta));
        double PosSX = r * Math.cos(theta + phi)+x;
        double PosSY = r * Math.sin(theta + phi)+y;

        double k = (Math.PI/2 - sens_j + phi);

        if (sens_D_nsew == 'n'){
            k+=0;
        }else if (sens_D_nsew =='s'){
            k+=Math.PI;
        }else if (sens_D_nsew =='e'){
            k-=Math.PI/2;
        }else if (sens_D_nsew =='w'){
            k+=Math.PI/2;
        }else{
            System.out.println("Please input where the sensor is pointing relative to the robot, only allow n(north), s(south), e(east), w(west)");
        }
        k = (k + Math.PI * 4)%(Math.PI*2);
//        System.out.println("PosSX: " + PosSX );
//        System.out.println("PosSY: " + PosSY );
        System.out.println("k deg: " + k/Math.PI * 180+ '\n'+ "k rad: " + k  );
        //  System.out.println("k?: " + (k-Math.PI)/Math.PI * 180  );

        double finallength = -1;
        int ret = 0;

        double field1 = 141;
        double field2 = 0;
        if (k == Math.PI){
            finallength = (field2 - PosSX) / (Math.cos(k));
            ret = 1;
        }
        else if (k== 0){
            finallength = (field1 - PosSX) / (Math.cos(k));
            ret = 2;
        }
        else if (k==3*Math.PI/2){
            finallength = (field2 - PosSY) / (Math.sin(k));
            ret = 3;
        }
        else if (k== Math.PI/2){
            finallength =  (field1 - PosSY) / (Math.sin(k));
            ret = 4;
        }
        else {
            if (k > 0 && k < Math.PI/2){
                double length2  = (field1 - PosSX) / (Math.cos(k));
                double length4  = (field1 - PosSY) / (Math.sin(k));
                double length2y = PosSY+ Math.sin(k)*length2;
                //double length4x = PosSX + Math.cos(k)*length4;
                if (length2y>=0 && length2y <=141){
                    finallength = length2;
                    ret = 2;
                } else {
                    finallength = length4;
                    ret = 4;
                }
            }else if (k > Math.PI/2 && k < Math.PI){
                double length1  = (field2 - PosSX) / (Math.cos(k));
                double length4  = (field1 - PosSY) / (Math.sin(k));
                double length1y = PosSY+ Math.sin(k)*length1;
                //double length4x = PosSX + Math.cos(k)*length4;
                if (length1y>=0 && length1y <=141){
                    finallength = length1;
                    ret = 1;
                }else {
                    finallength = length4;
                    ret = 4;
                }

            }else if (k > Math.PI && k < 3*Math.PI/2){
                double length1  = (field2 - PosSX) / (Math.cos(k));
                double length3  = (field2 - PosSY) / (Math.sin(k));
                double length1y = PosSY+ Math.sin(k)*length1;
                //double length3x = PosSX + Math.cos(k)*length3;
                if (length1y>=0 && length1y <=141){
                    finallength = length1;
                    ret = 1;
                }else {
                    finallength = length3;
                    ret = 3;
                }
            }else if (k > 3*Math.PI/2 && k < Math.PI*2){
                double length2  = (field1 - PosSX) / (Math.cos(k));
                double length3  = (field2 - PosSY) / (Math.sin(k));
                double length2y = PosSY+ Math.sin(k)*length2;
                //double length3x = PosSX + Math.cos(k)*length3;
                if (length2y>=0 && length2y <=141){
                    finallength = length2;
                    ret = 2;
                }else {
                    finallength = length3;
                    ret = 3;
                }
            }else {System.out.println("shouldn't be like this, today you fail, tomorrow try again.");}

        }
        System.out.println("finallength = " + finallength);
        System.out.println("ret = " + ret);
        System.out.println("errort = " + (ideal-finallength));
        return ((ideal-finallength)*(ideal-finallength));
    }

    public void nelder(double x, double y, double phi, final double ideal1, final double ideal2, final double ideal3) {
        SimplexOptimizer optimizer = new SimplexOptimizer(1e-10, 1e-30);
        MultivariateFunction selectedEquation;
//        if (equation == 1) {
        selectedEquation = new EquationOne(ideal1,ideal2,ideal3);
        final PointValuePair optimum =
                optimizer.optimize(
                        new MaxEval(10000),
                        new ObjectiveFunction(selectedEquation),
                        GoalType.MINIMIZE,
                        new InitialGuess(new double[]{x, y, phi}),
                        new NelderMeadSimplex(new double[]{1, 1, 1}));

        System.out.println(Arrays.toString(optimum.getPoint()) + " : "
                + optimum.getValue());
        pos = new Vector3(optimum.getPoint()[0],optimum.getPoint()[1],optimum.getPoint()[2]);
//        }
    } /*else if (equation == 2) {
             selectedEquation = new EquationTwo(ideal, sens_x, sens_y, sens_j,sens_D_nsew);
            final PointValuePair optimum =
                    optimizer.optimize(
                            new MaxEval(10000),
                            new ObjectiveFunction(selectedEquation),
                            GoalType.MINIMIZE,
                            new InitialGuess(new double[]{x, phi}),
                            new NelderMeadSimplex(new double[]{1, 1}));

            System.out.println(Arrays.toString(optimum.getPoint()) + " : "
                    + optimum.getValue());
        } else if (equation == 3) {
             selectedEquation = new EquationThree(ideal, sens_x, sens_y, sens_j,sens_D_nsew);
            final PointValuePair optimum =
                    optimizer.optimize(
                            new MaxEval(10000),
                            new ObjectiveFunction(selectedEquation),
                            GoalType.MINIMIZE,
                            new InitialGuess(new double[]{y, phi}),
                            new NelderMeadSimplex(new double[]{1, 1}));

            System.out.println(Arrays.toString(optimum.getPoint()) + " : "
                    + optimum.getValue());
        } else if (equation == 4) {
             selectedEquation = new EquationFour(ideal, sens_x, sens_y, sens_j,sens_D_nsew);
            final PointValuePair optimum =
                    optimizer.optimize(
                            new MaxEval(10000),
                            new ObjectiveFunction(selectedEquation),
                            GoalType.MINIMIZE,
                            new InitialGuess(new double[]{y, phi}),
                            new NelderMeadSimplex(new double[]{1, 1}));

            System.out.println(Arrays.toString(optimum.getPoint()) + " : "
                    + optimum.getValue());
    }
*/
    private class EquationOne implements MultivariateFunction {
        double ideal1;
        double ideal2;
        double ideal3;
        double sens_x;
        double sens_y;

        public EquationOne(final double ideal1, final double ideal2, final double ideal3){
            this.ideal1 = ideal1;
            this.ideal2 = ideal2;
            this.ideal3 = ideal3;
        }
        public double value(double[] variables) {

            double error = tripleInitial(variables[0],variables[1],variables[3],ideal1,ideal2,ideal3);
            return error;
            /*final double x = variables[0];
            final double phi = variables[1];

            double r     = Math.atan(sens_y/sens_x);
            double theta = Math.sqrt(sens_x * sens_x + sens_y * sens_y);

            double PosSX = r * Math.cos(theta + phi)+x;
            //double PosSY = r * Math.sin(theta + phi)+y;

            double k = (Math.PI/2 + sens_j - phi)-Math.PI;



            //return Math.pow((((141 - x - sens_x * Math.cos(phi) - sens_y * Math.sin(phi)) / (Math.cos(phi + sens_j))-ideal)),2);
            return Math.pow((141 - PosSX)/Math.cos(k)-ideal,2);*/
        }
    }


   /* private static class EquationTwo implements MultivariateFunction {
        double ideal;
        double sens_x;
        double sens_y;
        double sens_j;
        char sens_D_nsew;
        public EquationTwo(final double ideal, final double sens_x, final double sens_y, final double sens_j, final char sens_D_nsew){
            this.ideal = ideal;
            this.sens_x = sens_x;
            this.sens_y = sens_y;
            this.sens_j = sens_j;
            this.sens_D_nsew = sens_D_nsew;
        }
        public double value(double[] variables) {
            final double x = variables[0];
            final double phi = variables[1];

            double r     = Math.atan(sens_y/sens_x);
            double theta = Math.sqrt(sens_x * sens_x + sens_y * sens_y);

            double PosSX = r * Math.cos(theta + phi)+x;
            //double PosSY = r * Math.sin(theta + phi)+y;

            double k =  (Math.PI/2 + sens_j - phi)-Math.PI;

            //return Math.pow((((0 - x - sens_x * Math.cos(phi) - sens_y * Math.sin(phi)) / (Math.cos(phi + sens_j))-ideal)),2);
            return Math.pow(-PosSX/Math.cos(k)-ideal, 2);
        }
    }
    private static class EquationThree implements MultivariateFunction {
        double ideal;
        double sens_x;
        double sens_y;
        double sens_j;

        char sens_D_nsew;
        public EquationThree(final double ideal, final double sens_x, final double sens_y, final double sens_j, final char sens_D_nsew){
            this.ideal = ideal;
            this.sens_x = sens_x;
            this.sens_y = sens_y;
            this.sens_j = sens_j;
            this.sens_D_nsew = sens_D_nsew;
        }
        public double value(double[] variables) {
            final double y = variables[0];
            final double phi = variables[1];

            double r     = Math.atan(sens_y/sens_x);
            double theta = Math.sqrt(sens_x * sens_x + sens_y * sens_y);

            //double PosSX = r * Math.cos(theta + phi)+x;
            double PosSY = r * Math.sin(theta + phi)+y;
            double k =  (Math.PI/2 + sens_j - phi) - Math.PI;
          //return Math.pow((((141 - y - sens_x * Math.sin(phi) - sens_y * Math.cos(phi)) / (Math.sin(phi + sens_j)))-ideal),2);
            return Math.pow((141 - PosSY) / Math.sin(k)-ideal,2 );
        }
    }
    private static class EquationFour implements MultivariateFunction {
        double ideal;
        double sens_x;
        double sens_y;
        double sens_j;

        char sens_D_nsew;
        public EquationFour(final double ideal, final double sens_x, final double sens_y, final double sens_j, final char sens_D_nsew){
            this.ideal = ideal;
            this.sens_x = sens_x;
            this.sens_y = sens_y;
            this.sens_j = sens_j;
            this.sens_D_nsew = sens_D_nsew;
        }
        public double value(double[] variables) {
            final double y = variables[0];
            final double phi = variables[1];
            double r     = Math.atan(sens_y/sens_x);
            double theta = Math.sqrt(sens_x * sens_x + sens_y * sens_y);

            //double PosSX = r * Math.cos(theta + phi)+x;
            double PosSY = r * Math.sin(theta + phi)+y;

            double k =  (Math.PI/2 + sens_j - phi)-Math.PI;
            //return Math.pow((((0 - y - sens_x * Math.sin(phi) - sens_y * Math.cos(phi)) / (Math.sin(phi + sens_j)))-ideal),2);
            return Math.pow(- PosSY / Math.sin(k)-ideal,2 );
        */
}
