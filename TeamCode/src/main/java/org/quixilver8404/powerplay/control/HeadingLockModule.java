package org.quixilver8404.powerplay.control;

import org.quixilver8404.powerplay.util.Tunable;
import org.quixilver8404.powerplay.util.measurement.Angle;

public class HeadingLockModule {

    @Tunable
    private final PIDController rotationPID = new PIDController(6.65 / 90.0, 0, 2.0, 0);
    private double targetHeadingDeg;
    private boolean enabledPID = true;

    private double lastRotationPIDOutput = 0;

    public HeadingLockModule(Angle startingAngle) {
        targetHeadingDeg = startingAngle.getStandard(Angle.Unit.DEGREES);
    }

    public synchronized void update(BaseRobot baseRobot) {
        long currentTimeMillis = baseRobot.hwCollection.clock.getRunningTimeMillis();

        if (enabledPID) {
            double currentHeadingDeg = baseRobot.navModule.getHeading().getStandard(Angle.Unit.DEGREES);
            double angleOffsetDeg = (currentHeadingDeg - targetHeadingDeg + 3600) % 360;
            if (angleOffsetDeg > 180) {
                angleOffsetDeg -= 360;
            }
            double rotationPower = rotationPID.loop(angleOffsetDeg, currentTimeMillis);
            if (Math.abs(rotationPower) > 1) {
                rotationPID.resetIntegral();
            }
            baseRobot.driveModule.setTargetRotatePower(rotationPower);
            lastRotationPIDOutput = rotationPower;
        } else {
            rotationPID.reset();
            lastRotationPIDOutput = 0;
        }
    }

    public synchronized void enablePID(BaseRobot baseRobot) {
        if (!enabledPID) {
            enabledPID = true;
            setTargetHeading(baseRobot.navModule.getHeading());
        }
    }

    public synchronized void setCurHeading(Angle heading, BaseRobot robot) {
        enabledPID = false;
        robot.navModule.setCurHeading(heading);
        targetHeadingDeg = heading.getStandard(Angle.Unit.DEGREES);
        rotationPID.reset();
        enabledPID = true;
    }

    public synchronized void disablePID() {
        enabledPID = false;
    }

    public synchronized void setTargetHeading(Angle targetHeading) {
        enabledPID = true;
        targetHeadingDeg = targetHeading.getStandard(Angle.Unit.DEGREES);
    }

    public synchronized Angle getTargetHeading() {
        return new Angle(targetHeadingDeg, Angle.Unit.DEGREES);
    }

    public synchronized double getLastRotationPIDOutput() {
        return lastRotationPIDOutput;
    }
}
