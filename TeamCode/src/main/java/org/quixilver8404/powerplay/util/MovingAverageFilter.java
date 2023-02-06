package org.quixilver8404.powerplay.util;

import org.quixilver8404.powerplay.control.BaseRobot;

import java.sql.Array;
import java.util.ArrayList;

public class MovingAverageFilter {
    int window = 20;
    BaseRobot robot;
    ArrayList<Double> xlist = new ArrayList<>();
    ArrayList<Double> ylist = new ArrayList<>();
    ArrayList<Double> thetalist = new ArrayList<>();


    public MovingAverageFilter(BaseRobot robot) {
        this.robot = robot;
    }

    public void addOdoX() {
        xlist.add(robot.poseModule.getPosX());
        if (xlist.size() > window){
            xlist.remove(0);
        }
    }
    public void addOdoY() {
        ylist.add(robot.poseModule.getPosY());
        if (ylist.size() > window){
            ylist.remove(0);
        }
    }
    public void addOdoTheta() {
        thetalist.add(robot.poseModule.getPosTheta());
        if (thetalist.size() > window){
            thetalist.remove(0);
        }
    }
    public double getAverageX() {
        double sum = 0;
        for (double x : xlist){
            sum += x;
        }
        return sum/xlist.size();
    }
    public double getAverageY() {
        double sum = 0;
        for (double y : ylist){
            sum += y;
        }
        return sum/ylist.size();
    }
    public double getAverageTheta() {
        double sum = 0;
        for (double theta : thetalist){
            sum += theta;
        }
        return (sum/thetalist.size() * 180/Math.PI) % 360;
    }
    public void addUltraX() {
        xlist = new ArrayList<>();
        xlist.add(robot.mSonicModule.getPosX());
    }
    public void addUltraY() {
        ylist = new ArrayList<>();
        ylist.add(robot.mSonicModule.getPosY());
    }
    public void addUltraTheta() {
        thetalist = new ArrayList<>();
        thetalist.add(robot.mSonicModule.getPosTheta());
    }
    public void addControlIMUTheta() {
        thetalist.add(robot.hwCollection.controlIMU.getYaw());
        if (thetalist.size() > window){
            thetalist.remove(0);
        }
    }

}
