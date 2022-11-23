package org.quixilver8404.powerplaycode.control.base.modules;

import org.quixilver8404.powerplaycode.control.base.HardwareCollection;
import org.quixilver8404.powerplaycode.control.base.Robot;

public class DriveModule {

    protected enum MovementMode {
        INTRINSIC, AUTON, DISABLED
    }

    protected double targetPowerX;
    protected double targetPowerY;
    protected double targetRotatePower;
    protected double[] powerSettings;
    protected double[] powerCorrections;

    protected MovementMode movementMode;

    /**
     * Intializes variables
     */
    public DriveModule() {
        targetPowerX = 0;
        targetPowerY = 0;
        targetRotatePower = 0;
        powerSettings = new double[]{0,0,0,0};
        powerCorrections = new double[]{0,0,0,0};
        movementMode = MovementMode.INTRINSIC;

        //colorful colors and such
//        hwCollection.colorCommandControl = new LynxSetModuleLEDColorCommand(hwCollection.revHub1);
//        hwCollection.colorCommandExpansion = new LynxSetModuleLEDColorCommand(hwCollection.revHub2);
    }

    /**
     * Sets the power of the motors to be between -1 and 1 based on how hard the trigger is pressed
     * @param robot - a reference to the robot itself
     * @param hwCollection - references the actual hardware in this case the motors
     */
    public synchronized void update(final Robot robot, final HardwareCollection hwCollection) {
        //colorfullness


//        Log.d("Update", "DriveModule");
        double curHeadingRad = robot.poseModule.getPos().theta();

        // these are relative powers
        double intrinsicPowerX = 0;
        double intrinsicPowerY = 0;

        switch (movementMode) {
            case INTRINSIC:
                intrinsicPowerX = targetPowerX;
                intrinsicPowerY = targetPowerY;
                break;
            case DISABLED:
                return;
        }

        double powerFR = intrinsicPowerY - intrinsicPowerX - targetRotatePower;
        double powerFL = intrinsicPowerY + intrinsicPowerX + targetRotatePower;
        double powerBL = intrinsicPowerY - intrinsicPowerX + targetRotatePower;
        double powerBR = intrinsicPowerY + intrinsicPowerX - targetRotatePower;

        if (movementMode == MovementMode.AUTON) {
            powerFR = powerSettings[0] + powerCorrections[0];
            powerFL = powerSettings[1] + powerCorrections[1];
            powerBL = powerSettings[2] + powerCorrections[2];
            powerBR = powerSettings[3] + powerCorrections[3];
        }

        double maxPower = Math.max(Math.abs(powerFL), Math.abs(powerFR));
        maxPower = Math.max(maxPower, Math.abs(powerBL));
        maxPower = Math.max(maxPower, Math.abs(powerBR));

        if (maxPower > 1) {
            powerFR /= maxPower;
            powerFL /= maxPower;
            powerBL /= maxPower;
            powerBR /= maxPower;
        }

//        Log.d("Movement mode", movementMode.toString());
        hwCollection.driveMotorFR.setPower(powerFR);
        hwCollection.driveMotorFL.setPower(powerFL);
        hwCollection.driveMotorBL.setPower(powerBL);
        hwCollection.driveMotorBR.setPower(powerBR);
    }

    /**
     * Sets the target power to what is inputed and the movement mode to extrinsic
     * @param targetPowerX - double of the target power along the x axis
     * @param targetPowerY - double of the target power along the y axis
     */
    public synchronized void setIntrinsicTargetPower(final double targetPowerX, final double targetPowerY) {
//        Log.d("Changing target power", targetPowerX + ", " + targetPowerX);
        this.targetPowerX = targetPowerX;
        this.targetPowerY = targetPowerY;
        movementMode = MovementMode.INTRINSIC;
    }

    /**
     * Sets the target vector using a specific power and angle
     * @param targetPower - double of the target power
     * @param heading - angle of the power vector
     */
    public synchronized void setIntrinsicTargetVector(final double targetPower, final double heading) {
//        Log.d("Changing target power", ""+targetPower);
        this.targetPowerX = targetPower * Math.cos(heading+Math.PI/2);
        this.targetPowerY = targetPower * Math.sin(heading+Math.PI/2);
        movementMode = MovementMode.INTRINSIC;
    }

    /**
     * Sets the target rotate power to the input
     * @param targetRotatePower - double of target power for rotation
     */
    public synchronized void setTargetRotatePower(final double targetRotatePower) {
        this.targetRotatePower = targetRotatePower;
    }

    public synchronized void setPowerSettings(final double[] powerSettings) {
        this.powerSettings = powerSettings;
        movementMode = MovementMode.AUTON;
    }

    public synchronized void setPowerCorrections(final double[] powerCorrections) {
        this.powerCorrections = powerCorrections;
    }

    /**
     * Stops movement
     */
    public synchronized void cancelAllMovement() {
        targetPowerX = 0;
        targetPowerY = 0;
        targetRotatePower = 0;
    }

    public MovementMode getMovementMode() {
        return movementMode;
    }

    public double[] getPowerSettings() {
        return powerSettings;
    }

    public void enableAuton() {
        movementMode = MovementMode.AUTON;
    }

    public synchronized void disable() {
        movementMode = MovementMode.DISABLED;
    }

    public synchronized void enable() {
        movementMode = MovementMode.INTRINSIC;
    }
}
