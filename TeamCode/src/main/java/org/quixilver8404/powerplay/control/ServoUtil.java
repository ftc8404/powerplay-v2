package org.quixilver8404.powerplay.control;

import org.quixilver8404.powerplay.util.measurement.Angle;

public class ServoUtil {

    /**
     * Givens the calibrated angle-position relationship, this method gives the position for any angle.
     * For values outside the servo's range, the wraparound determines whether to choose the 0 or 1 position.
     */
    public static double angleToPosition(Angle angle1, double position1, Angle angle2, double position2, Angle wrapAround, Angle targetAngle) {
        double angle1Rad = angle1.getStandard(Angle.Unit.RADIANS);
        double angle2Rad = angle2.getStandard(Angle.Unit.RADIANS);
        double wrapAroundRad = wrapAround.getStandard(Angle.Unit.RADIANS);

        double angle1FromWrapAroundRad = (angle1Rad - wrapAroundRad + Math.PI * 4) % (Math.PI * 2);
        double angle2FromWrapAroundRad = (angle2Rad - wrapAroundRad + Math.PI * 4) % (Math.PI * 2);

        if (angle2FromWrapAroundRad > angle1FromWrapAroundRad) {
            while (angle1Rad < wrapAroundRad) {
                angle1Rad += Math.PI * 2;
            }
            while (angle2Rad < angle1Rad) {
                angle2Rad += Math.PI * 2;
            }
        } else {
            while (angle2Rad < wrapAroundRad) {
                angle2Rad += Math.PI * 2;
            }
            while (angle1Rad < angle2Rad) {
                angle1Rad += Math.PI * 2;
            }
        }
        double targetAngleRad = targetAngle.getStandard(Angle.Unit.RADIANS);
        while (targetAngleRad < wrapAroundRad) {
            targetAngleRad += Math.PI * 2;
        }
        return position1 + (position2 - position1) * (targetAngleRad - angle1Rad) / (angle2Rad - angle1Rad);
    }
}
