package org.quixilver8404.powerplaycode.util;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class SensorTransformation {

    public final ArrayRealVector p_tilde;
    public final double rotation;
    public final Array2DRowRealMatrix m_tilde_inv;

    public SensorTransformation(final Vector3 posRelativeToRobot) {
        p_tilde = new ArrayRealVector(new double[]{posRelativeToRobot.x(), posRelativeToRobot.y()});
        rotation = posRelativeToRobot.theta();
        m_tilde_inv = eulerRotationMatrixZ(-posRelativeToRobot.theta());
    }

    public Vector3 robotPosFromSensorPos(final Vector3 sensorPosVector) {
        final ArrayRealVector p_hat = new ArrayRealVector(new double[]{sensorPosVector.x(), sensorPosVector.y()});
        final Array2DRowRealMatrix m_hat = eulerRotationMatrixZ(sensorPosVector.theta());
        final RealVector p = p_hat.subtract(m_tilde_inv.operate(m_hat.operate(p_tilde)));
        return new Vector3(p.getEntry(0), p.getEntry(1), sensorPosVector.theta()-rotation);
    }

    public Vector3 sensorPosFromRobotPos(final Vector3 robotPosVector) {
        final ArrayRealVector p = new ArrayRealVector(new double[]{robotPosVector.x(), robotPosVector.y()});
        final Array2DRowRealMatrix m = eulerRotationMatrixZ(robotPosVector.theta());
        final RealVector p_hat = p.add(m.operate(p_tilde));
        return new Vector3(p_hat.getEntry(0), p_hat.getEntry(1), robotPosVector.theta()+rotation);//m+m_tilde=m_hat
    }

    public static Vector3 calibrateSensorPosition(final Vector3 robotPosVector, final Vector3 sensorPosVector) {
        final ArrayRealVector p = new ArrayRealVector(new double[]{robotPosVector.x(), robotPosVector.y()});
        final Array2DRowRealMatrix m_inv = eulerRotationMatrixZ(-robotPosVector.theta());
        final ArrayRealVector p_hat = new ArrayRealVector(new double[]{sensorPosVector.x(), sensorPosVector.y()});
        final RealVector p_tilde = m_inv.operate(p_hat.subtract(p));
        return new Vector3(p_tilde.getEntry(0), p_tilde.getEntry(1), sensorPosVector.theta()-robotPosVector.theta());
    }

    public static Array2DRowRealMatrix eulerRotationMatrixZ(final double r_z) {
        return new Array2DRowRealMatrix(new double[][] {
                {Math.cos(r_z), -Math.sin(r_z)},
                {Math.sin(r_z), Math.cos(r_z)}}
        );
    }
}
