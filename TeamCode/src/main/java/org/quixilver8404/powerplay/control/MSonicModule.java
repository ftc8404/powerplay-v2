package org.quixilver8404.powerplay.control;

import com.qualcomm.robotcore.robot.Robot;

import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.NelderMeadSimplex;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.SimplexOptimizer;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.quixilver8404.powerplay.util.Vector3;

import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;

public class MSonicModule {
    BaseRobot robot;

//    double[] ultraFront = polar(7.131, 6.767, 0, 'w');  //config
//    double[] ultraRight = polar(1.831, 7.567, 0, 'n');  //config
//    double[] ultraLeft = polar(1.831, -6.233, 0, 's');  //config
//    double[] distFront = polar(8.931, -3.833, 0, 'w');  //config
//    double[] distLeft = polar(4.931, -6.333, 0, 's');  //config
//    double[] distRight = polar(4.831, 7.667, 0, 'n');  //config
//    double[] distBack = polar(-6.969, -3.733, 0, 'e');  //config
//    double[] zero = new double[3];
//    double[][] sensList = new double[][]{zero,ultraFront,ultraLeft,ultraRight,distFront,distLeft,distRight,distBack};
//
//    double smallLength = 12;
    Vector3 prevPos = new Vector3();

    int senConfig = 0;


    public MSonicModule (BaseRobot robot) {
        this.robot = robot;
    }

    public synchronized void update(){
        Vector3 v = new Vector3();
        if (senConfig == 1 && robot.poseModule.getPos() != null) {
            double y = (141 - ((robot.hwCollection.ultraLeft.getDistance(DistanceUnit.INCH) + 6.233)+(robot.hwCollection.dLeft.getDistance(DistanceUnit.INCH)+6.333))/2)*0.0254;
            double x = (robot.hwCollection.dBack.getDistance(DistanceUnit.INCH)+6.969)*0.0254;
            if (robot.hwCollection.ultraLeft.getDistance(DistanceUnit.INCH) < 12 || robot.hwCollection.dLeft.getDistance(DistanceUnit.INCH) < 12){
                y = robot.poseModule.getPosY();
            }
            if (robot.hwCollection.dBack.getDistance(DistanceUnit.INCH) < 12){
                x = robot.poseModule.getPosX();
            }
            v = new Vector3(x,y,robot.poseModule.getPosTheta());
        } else if (senConfig == 2 && robot.poseModule.getPos() != null) {
            double y = (141 - ((robot.hwCollection.ultraFront.getDistance(DistanceUnit.INCH) + 7.131)+(robot.hwCollection.dFront.getDistance(DistanceUnit.INCH)+8.931))/2)*0.0254;
            double x = (((robot.hwCollection.ultraLeft.getDistance(DistanceUnit.INCH) + 6.233)+(robot.hwCollection.dLeft.getDistance(DistanceUnit.INCH)+6.333))/2)*0.0254;
            if (robot.hwCollection.ultraFront.getDistance(DistanceUnit.INCH) < 12 || robot.hwCollection.dFront.getDistance(DistanceUnit.INCH) < 12){
                y = robot.poseModule.getPosY();
            }
            if (robot.hwCollection.ultraLeft.getDistance(DistanceUnit.INCH) < 12 || robot.hwCollection.dLeft.getDistance(DistanceUnit.INCH) < 12){
                x = robot.poseModule.getPosX();
            }
            v = new Vector3(x,y,robot.poseModule.getPosTheta());
        } else if (senConfig == 3 && robot.poseModule.getPos() != null) {
            double y = (((robot.hwCollection.ultraRight.getDistance(DistanceUnit.INCH) + 7.567)+(robot.hwCollection.dRight.getDistance(DistanceUnit.INCH)+7.667))/2)*0.0254;
            double x = (robot.hwCollection.dBack.getDistance(DistanceUnit.INCH)+6.969)*0.0254;
            if (robot.hwCollection.ultraRight.getDistance(DistanceUnit.INCH) < 12 || robot.hwCollection.dRight.getDistance(DistanceUnit.INCH) < 12){
                y = robot.poseModule.getPosY();
            }
            if (robot.hwCollection.dBack.getDistance(DistanceUnit.INCH) < 12){
                x = robot.poseModule.getPosX();
            }
            v = new Vector3(x,y,robot.poseModule.getPosTheta());
        } else if (senConfig == 4 && robot.poseModule.getPos() != null) {
            double y = (((robot.hwCollection.ultraFront.getDistance(DistanceUnit.INCH) + 7.131)+(robot.hwCollection.dFront.getDistance(DistanceUnit.INCH)+8.931))/2)*0.0254;
            double x = (((robot.hwCollection.ultraRight.getDistance(DistanceUnit.INCH) + 7.567)+(robot.hwCollection.dRight.getDistance(DistanceUnit.INCH)+7.667))/2)*0.0254;
            if (robot.hwCollection.ultraFront.getDistance(DistanceUnit.INCH) < 12 || robot.hwCollection.dFront.getDistance(DistanceUnit.INCH) < 12){
                y = robot.poseModule.getPosY();
            }
            if (robot.hwCollection.ultraRight.getDistance(DistanceUnit.INCH) < 12 || robot.hwCollection.dRight.getDistance(DistanceUnit.INCH) < 12){
                x = robot.poseModule.getPosX();
            }
            v = new Vector3(x,y,robot.poseModule.getPosTheta());
        } if (!v.isEqual(new Vector3()) && !v.isEqual(prevPos)){
            System.out.println("Setting leftBefore pos");
            robot.poseModule.setPos(v);
            prevPos = v;

        }
    }
    public synchronized void setConfig(int config){
        senConfig = config;
    }
//    public synchronized Vector3 result(double x, double y, double phi, final double ideal1, final double[] polar) {
//        //double equation = initial(x, y, phi, ideal1, ideal2, ideal3);
//        //System.out.println(equation);
//        //CONFIGGGG
//        // tripleInitial(x, y, phi, ideal1,ideal2,ideal3, pol1,pol2,pol3);
//        double eqULeft = initial(x,y,phi,ideal1,polar[0],polar[1],polar[2]);
//        double posX = -1;
//        double posY = -1;
//        polar[2] += phi;
//        polar[2] = (polar[2] + Math.PI * 4) % (Math.PI * 2);
//        if (eqULeft == 0){
//            posX = -polar[0]*Math.cos(polar[1]+phi)-Math.cos(polar[2])*ideal1;
//        } else if (eqULeft == 1){
//            posX = 141-polar[0]*Math.cos(polar[1]+phi)-Math.cos(polar[2])*ideal1;
//        } else if (eqULeft == 2){
//            posY = -polar[0]*Math.sin(polar[1]+phi)-Math.sin(polar[2])*ideal1;
//        } else if (eqULeft == 3){
//            posY = 141-polar[0]*Math.sin(polar[1]+phi)-Math.sin(polar[2])*ideal1;
//        }
//        return new Vector3(posX,posY,phi);
//    }
//    public synchronized Vector3 leftAfter(double x, double y, double phi, final double ideal1, final double ideal2, final double ideal3, final double ideal4) {
//        //double equation = initial(x, y, phi, ideal1, ideal2, ideal3);
//        //System.out.println(equation);
//        //CONFIGGGG
//        // tripleInitial(x, y, phi, ideal1,ideal2,ideal3, pol1,pol2,pol3);
//        double[] pos = nelder(x, y, phi, ideal1, ideal2, ideal3, ideal4, ultraLeft, distLeft, ultraFront, distFront);
//        return new Vector3(pos[0], pos[1], pos[2]);
//    }
//    public synchronized Vector3 rightBefore(double x, double y, double phi, final double ideal1, final double ideal2, final double ideal3, final double ideal4) {
//        //double equation = initial(x, y, phi, ideal1, ideal2, ideal3);
//        //System.out.println(equation);
//        //CONFIGGGG
//        // tripleInitial(x, y, phi, ideal1,ideal2,ideal3, pol1,pol2,pol3);
//        double[] pos = nelder(x, y, phi, ideal1, ideal2, ideal3, ideal4, ultraRight, distRight, distBack, zero);
//        return new Vector3(pos[0], pos[1], pos[2]);
//    }
//    public synchronized Vector3 rightAfter(double x, double y, double phi, final double ideal1, final double ideal2, final double ideal3, final double ideal4) {
//        //double equation = initial(x, y, phi, ideal1, ideal2, ideal3);
//        //System.out.println(equation);
//        //CONFIGGGG
//        // tripleInitial(x, y, phi, ideal1,ideal2,ideal3, pol1,pol2,pol3);
//        double[] pos = nelder(x, y, phi, ideal1, ideal2, ideal3, ideal4, ultraRight, distRight, ultraFront, distFront);
//        return new Vector3(pos[0], pos[1], pos[2]);
//    }
//
//    public synchronized double tripleInitial(double x, double y, double phi, double ideal1, double ideal2, double ideal3, double ideal4, double[] pol1, double[] pol2, double[] pol3, double[] pol4) {
//        //testing config
////        double s1 = initial(x,y,phi,ideal1,1,0, 0);//testing
////       // System.out.println(s1);
////        double s2 = initial(x,y,phi,ideal2,1,2*Math.PI/3, 2*Math.PI/3);//testing
////       // System.out.println(s2);
////        double s3 = initial(x,y,phi,ideal3,1,4*Math.PI/3, 4*Math.PI/3);//testing
////       // System.out.println(s3);
////        return s1+s2+s3;
//        double s1 = initial(x, y, phi, ideal1, pol1[0], pol1[1], pol1[2]);
//        double s2 = initial(x, y, phi, ideal2, pol2[0], pol2[1], pol2[2]);
//        double s3 = initial(x, y, phi, ideal3, pol3[0], pol3[1], pol3[2]);
//        double s4 = initial(x, y, phi, ideal4, pol4[0], pol4[1], pol4[2]);
//        return s1 + s2 + s3 + s4;
//    }
//
//    public synchronized double[] polar(final double sens_x, final double sens_y, final double sens_j, final char sens_D_nsew) {
//        double r = Math.sqrt(sens_x * sens_x + sens_y * sens_y);
//        //System.out.println("r: " + r );
//        double theta;
//        if (sens_x == 0) {
//            theta = (Math.signum(sens_y) * Math.PI / 2 + 2 * Math.PI) % Math.PI;
//        } else {
//            theta = Math.atan(sens_y / sens_x);
//        }
//        // System.out.println("theta: " + theta/Math.PI * 180 );
//        if (theta < 0) {
//            if (sens_y > 0) theta = Math.PI + theta;
//            else theta = theta + 2 * Math.PI;
//        }
//        double k = Math.PI / 2 - sens_j;
//        if (sens_D_nsew == 'n') {
//            k += 0;
//        } else if (sens_D_nsew == 's') {
//            k += Math.PI;
//        } else if (sens_D_nsew == 'e') {
//            k -= Math.PI / 2;
//        } else if (sens_D_nsew == 'w') {
//            k += Math.PI / 2;
//        } else {
//            System.out.println("Please input where the sensor is pointing relative to the robot, only allow n(north), s(south), e(east), w(west)");
//        }
//        double[] ret = {r, theta, k};
//        return ret;
//    }
//
//    public synchronized double initial(double x, double y, double phi, final double ideal, final double r, final double theta, double k) {
//        //System.out.println("theta: " + theta/Math.PI * 180 );
//        // System.out.println("sens_x = " + r*Math.cos(theta));
//        if (ideal < smallLength){
//            return -1;
//        }
//        double PosSX = r * Math.cos(theta + phi) + x;
//        double PosSY = r * Math.sin(theta + phi) + y;
//
//        k += phi;
//        k = (k + Math.PI * 4) % (Math.PI * 2);
////        System.out.println("PosSX: " + PosSX );
////        System.out.println("PosSY: " + PosSY );
//        //   System.out.println("k deg: " + k/Math.PI * 180+ '\n'+ "k rad: " + k  );
//        double field = 141;
//
//        double length1 = (-PosSX) / (Math.cos(k));
//        double length2 = (field - PosSX) / (Math.cos(k));
//        double length3 = (-PosSY) / (Math.sin(k));
//        double length4 = (field - PosSY) / (Math.sin(k));
//        double[] lengths = {length2, length3, length4};
//        double finallength = length1;
//        int eq = 0;
//
//        for (int t = 0; t < 3; t++) {
//            if (finallength < 0 || (lengths[t] < finallength && lengths[t] > 0)) {
//                finallength = lengths[t];
//                eq = t;
//            }
//        }
//
//        // System.out.println("finallength =  " + finallength  );
//        return eq;
//    }
//
//    public synchronized double[] nelder(double x, double y, double phi, final double ideal1, final double ideal2, final double ideal3, final double ideal4, double[] pol1, double[] pol2, double[] pol3, double[] pol4) {
//        SimplexOptimizer optimizer = new SimplexOptimizer(1e-10, 1e-30);
//        MultivariateFunction selectedEquation;
//        selectedEquation = new EquationOne(ideal1, ideal2, ideal3, ideal4, pol1, pol2, pol3, pol4);
//        final PointValuePair optimum =
//                optimizer.optimize(
//                        new MaxEval(141),
//                        new ObjectiveFunction(selectedEquation),
//                        GoalType.MINIMIZE,
//                        new InitialGuess(new double[]{x, y, phi}),
//                        new NelderMeadSimplex(new double[]{1, 1, 1}));
//
//        return optimum.getPoint();
//    }
//
//    private class EquationOne implements MultivariateFunction {
//        double ideal1;
//        double ideal2;
//        double ideal3;
//        double ideal4;
//        double[] pol1;
//        double[] pol2;
//        double[] pol3;
//        double[] pol4;
//
//        public EquationOne(final double ideal1, final double ideal2, final double ideal3,final double ideal4, double[] pol1, double[] pol2, double[] pol3, double[] pol4) {
//            this.ideal1 = ideal1;
//            this.ideal2 = ideal2;
//            this.ideal3 = ideal3;
//            this.ideal4 = ideal4;
//
//            this.pol1 = pol1;
//            this.pol2 = pol2;
//            this.pol3 = pol3;
//            this.pol4 = pol4;
//        }
//
//        public double value(double[] variables) {
//
//            double error = tripleInitial(variables[0], variables[1], variables[2], ideal1, ideal2, ideal3, ideal4, pol1, pol2, pol3, pol4);
//            return error;
//        }
//    }
}