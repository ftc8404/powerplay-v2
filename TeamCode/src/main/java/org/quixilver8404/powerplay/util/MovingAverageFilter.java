package org.quixilver8404.powerplay.util;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.quixilver8404.powerplay.control.BaseRobot;
import org.quixilver8404.powerplay.control.MSonicModule;

import java.sql.Array;
import java.util.ArrayList;

public class MovingAverageFilter {
    int window = 5;
    BaseRobot robot;
    ArrayList<Double> xlist = new ArrayList<>();
    ArrayList<Double> ylist = new ArrayList<>();
    ArrayList<Double> thetalist = new ArrayList<>();


    public MovingAverageFilter(BaseRobot robot) {
        this.robot = robot;
    }

    public synchronized void update(){
        addOdoX();
        addOdoY();
        addOdoTheta();

//        addUltraX();
//        addUltraY();
    }

    public synchronized void addOdoX() {
        xlist.add(robot.poseModule.getPosX());
        if (xlist.size() > window){
            xlist.remove(0);
        }
    }
    public synchronized void addOdoY() {
        ylist.add(robot.poseModule.getPosY());
        if (ylist.size() > window){
            ylist.remove(0);
        }
    }
    public synchronized void addOdoTheta() {
        thetalist.add(robot.poseModule.getPosTheta());
        if (thetalist.size() > window){
            thetalist.remove(0);
        }
    }
    public synchronized double getAverageX() {
        double sum = 0;
        for (double x : xlist){
            sum += x;
        }
        return sum/xlist.size();
    }
    public synchronized double getAverageY() {
        double sum = 0;
        for (double y : ylist){
            sum += y;
        }
        return sum/ylist.size();
    }
    public synchronized double getAverageTheta() {
        double sum = 0;
        for (double theta : thetalist){
            sum += theta;
        }
        return (sum/thetalist.size());
    }
    public synchronized void addUltraX() {
        double ultraX = MSonicModule.tripleThreat(
                robot.movingAverageFilter.getAverageX() * 39.37,
                robot.movingAverageFilter.getAverageY() * 39.37,
                robot.movingAverageFilter.getAverageTheta(),
                robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH),
                robot.hwCollection.ultraSonic2.getDistance(DistanceUnit.INCH),
                robot.hwCollection.ultraSonic3.getDistance(DistanceUnit.INCH))[0]*0.0254;
        if (ultraX > 0) {
            for (int i = 0; i < 1; i++) {
                xlist.add(ultraX);
                if (xlist.size() > window) {
                    xlist.remove(0);
                }
            }
        }
    }
    public synchronized void addUltraY() {
        double ultraY = MSonicModule.tripleThreat(
                robot.movingAverageFilter.getAverageX() * 39.37,
                robot.movingAverageFilter.getAverageY() * 39.37,
                robot.movingAverageFilter.getAverageTheta(),
                robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH),
                robot.hwCollection.ultraSonic2.getDistance(DistanceUnit.INCH),
                robot.hwCollection.ultraSonic3.getDistance(DistanceUnit.INCH))[1]*0.0254;
        if (ultraY > 0) {
            for (int i = 0; i < 1; i++) {
                ylist.add(ultraY);
                if (ylist.size() > window) {
                    ylist.remove(0);
                }
            }
        }
    }
    public void addUltraTheta() {
        double ultraTheta = MSonicModule.tripleThreat(
                robot.movingAverageFilter.getAverageX() * 39.37,
                robot.movingAverageFilter.getAverageY() * 39.37,
                robot.movingAverageFilter.getAverageTheta(),
                robot.hwCollection.ultraSonic1.getDistance(DistanceUnit.INCH),
                robot.hwCollection.ultraSonic2.getDistance(DistanceUnit.INCH),
                robot.hwCollection.ultraSonic3.getDistance(DistanceUnit.INCH))[2];
        for (int i = 0; i < 1; i++) {
            thetalist.add(ultraTheta);
            if (thetalist.size() > window) {
                thetalist.remove(0);
            }
        }
    }
    public void resetFilter(){
        xlist = new ArrayList<>();
        ylist = new ArrayList<>();
        thetalist = new ArrayList<>();
    }
//    public void addControlIMUTheta() {
//        thetalist.add(robot.hwCollection.controlIMU.getYaw());
//        if (thetalist.size() > window){
//            thetalist.remove(0);
//        }
//    }

}
