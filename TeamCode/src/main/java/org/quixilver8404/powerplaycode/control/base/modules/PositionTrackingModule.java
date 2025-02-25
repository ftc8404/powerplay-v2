package org.quixilver8404.powerplaycode.control.base.modules;

import android.util.Pair;

import org.quixilver8404.powerplaycode.control.base.Robot;
import org.quixilver8404.powerplaycode.control.base.modules.util.Odometry;
import org.quixilver8404.powerplaycode.util.Vector3;

import java.util.ArrayList;

public class PositionTrackingModule {

    public static final double ODOMETRY_ENCODER_M_PER_TICK = 0.035*Math.PI/8192 * 3.13055/3.105;
    public static final Vector3 ODOMETRY_1_POSITION = new Vector3(3.5*0.0254,5.75*0.0254,0);
    public static final Vector3 ODOMETRY_2_POSITION = new Vector3(-2.7*0.0254,6.25*0.0254,0);
    public static final Vector3 ODOMETRY_3_POSITION = new Vector3(-0.5*0.0254,-4*0.0254,Math.PI/2);
    public static final Vector3[] ODOMETRY_WHEEL_SETUP = new Vector3[]{ODOMETRY_1_POSITION, ODOMETRY_2_POSITION, ODOMETRY_3_POSITION};

    public enum PositionTrackingState {
        ODOMETRY, OFF
    }

    public enum PositionTrackingAction {
        SWITCH_TO_ODOMETRY, DISABLE
    }

    protected Vector3 pos;
    protected Vector3 vel;
    protected PositionTrackingState poseState;
    protected PositionTrackingAction poseAction;
    protected final Odometry odometry;

    public final ArrayList<Pair<Vector3, Double>> odoPositions = new ArrayList<>(1000);

    /**
     * Intitailizes position based on the input
     * @param pos - vector position
     */
    public PositionTrackingModule(final Vector3 pos) {
        this.pos = pos;
        vel = new Vector3();
        poseState = PositionTrackingState.ODOMETRY;
        poseAction = PositionTrackingAction.SWITCH_TO_ODOMETRY;
        odometry = new Odometry(ODOMETRY_WHEEL_SETUP, pos);
    }

    /**
     * Gets the position of the robot
     * @param robot - a reference to the robot itself
     */
    public synchronized void update(final Robot robot) {
        final double odo1;
        final double odo2;
        final double odo3;
        switch (poseState) {
            case ODOMETRY:
                odo1 = robot.hardwareCollection.odometryEncoder1.getDeltaPosition() * ODOMETRY_ENCODER_M_PER_TICK;
                odo2 = robot.hardwareCollection.odometryEncoder2.getDeltaPosition() * ODOMETRY_ENCODER_M_PER_TICK;
                odo3 = robot.hardwareCollection.odometryEncoder3.getDeltaPosition() * ODOMETRY_ENCODER_M_PER_TICK;
                odometry.update(new double[]{odo1, odo2, odo3}, robot.hardwareCollection.clock.getDeltaTimeMS()/1000d, pos.theta());
                pos = pos.addVector(odometry.getDeltaPos());
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
