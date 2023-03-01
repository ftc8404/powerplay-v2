package org.quixilver8404.powerplay.control;

import android.util.Pair;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.quixilver8404.powerplay.util.Vector3;
import org.quixilver8404.powerplay.util.measurement.Angle;

import java.util.ArrayList;

public class PositionTrackingModule {

    public static final double ODOMETRY_ENCODER_M_PER_TICK = 0.035*Math.PI/8192;

    public static final Vector3 ODOMETRY_1_POSITION = new Vector3((-0.63) * 0.0254, (2.989) * 0.0254, Math.PI - 5*Math.PI/180);
    public static final Vector3 ODOMETRY_2_POSITION = new Vector3((-0.63) * 0.0254,(-4.5735) * 0.0254, Math.PI);
    public static final Vector3 ODOMETRY_3_POSITION = new Vector3((-6.1925) * 0.0254, (-0.1985) * 0.0254,Math.PI/2 * 1.01);
    public static final Vector3[] ODOMETRY_WHEEL_SETUP = new Vector3[]{ODOMETRY_1_POSITION, ODOMETRY_2_POSITION, ODOMETRY_3_POSITION};

    public enum PositionTrackingState {
        ODOMETRY, OFF
    }

    public enum PositionTrackingAction {
        SWITCH_TO_ODOMETRY, DISABLE
    }

    protected Vector3 pos;
    protected Vector3 startpos;
    protected Vector3 vel;
    protected PositionTrackingState poseState;
    protected PositionTrackingAction poseAction;
    protected final Odometry odometry;

    public final ArrayList<Pair<Vector3, Double>> odoPositions = new ArrayList<>(1000);

    /**
     * Initializes position based on the input
     * @param pos - vector position
     */
    public PositionTrackingModule(final Vector3 pos) {
        this.pos = pos;
        startpos = pos;
        vel = new Vector3();
        poseState = PositionTrackingState.ODOMETRY;
        poseAction = PositionTrackingAction.SWITCH_TO_ODOMETRY;
        odometry = new Odometry(ODOMETRY_WHEEL_SETUP, pos);
    }

    /**
     * Gets the position of the robot
     * @param robot - a reference to the robot itself
     */
    public synchronized void update(final BaseRobot robot) {
        final double odo1;
        final double odo2;
        final double odo3;
//        YawPitchRollAngles orientation = robot.hwCollection.imu.getRobotYawPitchRollAngles();
        switch (poseState) {
            case ODOMETRY:
                odo1 = robot.hwCollection.driveEncoderLeft.getDeltaPosition() * ODOMETRY_ENCODER_M_PER_TICK;
                odo2 = robot.hwCollection.driveEncoderRight.getDeltaPosition() * ODOMETRY_ENCODER_M_PER_TICK;
                odo3 = robot.hwCollection.driveEncoderCenter.getDeltaPosition() * ODOMETRY_ENCODER_M_PER_TICK;
                odometry.update(new double[]{odo1, odo2, odo3}, robot.hwCollection.clock.getDeltaTimeMillis()/1000d, pos.theta());
                pos = pos.addVector(odometry.getDeltaPos());
                pos.setTheta((pos.theta()/*+orientation.getYaw(AngleUnit.RADIANS))/2*/));
//                pos = startpos.addVector(odometry.getPosition());
                vel = odometry.getVelocity();
                break;
            case OFF:
                return;

        }

        switch (poseAction) {
            case SWITCH_TO_ODOMETRY:
                if (poseState != PositionTrackingState.ODOMETRY) {
                    poseState = PositionTrackingState.ODOMETRY;
//                     robot.hardwareCollection.imu.stopAccelerationIntegration();
                }
                break;
            case DISABLE:
                if (poseState != PositionTrackingState.OFF) {
                    poseState = PositionTrackingState.OFF;
                }
                break;
        }
    }

    /**
     * Gives the current position
     * @return the current position
     */
    public synchronized Vector3 getPos() {
        return pos;
    }

    public synchronized double getPosX() {
        return pos.x();
    }

    public synchronized double getPosY() {
        return pos.y();
    }

    public synchronized double getPosTheta() {
        return pos.theta();
    }

    public synchronized void setStartPos(Vector3 position) {
        this.pos = position;
        startpos = position;
    }

    public synchronized void setPos(Vector3 position) {
        this.pos = position;
    }

    public synchronized Vector3 getVel() {
        return vel;
    }

    public synchronized void setAction(final PositionTrackingAction action) {
        poseAction = action;
        if (poseAction == PositionTrackingAction.SWITCH_TO_ODOMETRY) {
            odometry.setPosition(pos);
        }
    }

    public synchronized boolean disable() {
        if (poseState != PositionTrackingState.OFF) {
            poseAction = PositionTrackingAction.DISABLE;
            return true;
        }
        return false;
    }

    public synchronized Vector3 getRawDeltaPos() {
        return odometry.getRawDeltaPos();
    }

    public synchronized Vector3 getDeltaPos() {
        return odometry.getDeltaPos();
    }

    public PositionTrackingState getPoseState() {
        return poseState;
    }

}