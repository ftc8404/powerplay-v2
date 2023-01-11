package org.quixilver8404.skystone.control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.quixilver8404.skystone.util.measurement.Pose2D;

/**
 * Base robot class that facilitates basic navigation and control
 */
public abstract class BaseRobot {

    private final HardwareLoopThread hardwareLoopThread;

    protected final LinearOpMode opMode;

    protected final Telemetry telemetry;

    /**
     * Using this field outside of BaseRobot or its children should only be done for testing!
     */
    public final HardwareCollection hwCollection;

    // Submodules
    public final TaskModule taskModule;
    public final PathFollowModule pathFollowModule;
    public final HeadingLockModule headingLockModule;
    public final DiagnosticModule diagnosticModule;
    public final NavModule navModule;
    public final DriveModule driveModule;
    public final MSonicModule mSonicModule;

    protected long updateCount = -1; // will be incremented to 0 at the start of the first update
    protected final boolean prioritizeOdometry;

    BaseRobot(Pose2D startPose, boolean prioritizeOdometry, LinearOpMode opMode) {
        this.opMode = opMode;
        telemetry = opMode.telemetry;

        hwCollection = new HardwareCollection(opMode.hardwareMap);

        hardwareLoopThread = new HardwareLoopThread(this, opMode);

        diagnosticModule = new DiagnosticModule();
        taskModule = new TaskModule();
        pathFollowModule = new PathFollowModule();
        headingLockModule = new HeadingLockModule(startPose.heading);
        navModule = new NavModule(startPose, hwCollection);
        driveModule = new DriveModule();
        mSonicModule = new MSonicModule();

        this.prioritizeOdometry = prioritizeOdometry;
    }

    public void startHardwareLoop() {
        if (!hardwareLoopThread.isAlive()) {
            update();
            hardwareLoopThread.start();
        }
    }

    public void stopHardwareLoop() throws InterruptedException {
        if (hardwareLoopThread.isAlive()) {
            hardwareLoopThread.terminate();
            hardwareLoopThread.join();
        }
    }

    public void update() {
        updateCount++;

        hwCollection.refreshExpansionHubBulkData(); // refresh odometry readings
        hwCollection.clock.update();

        diagnosticModule.update(this, hwCollection);

        taskModule.update(this, hwCollection);

        pathFollowModule.update(this);
        headingLockModule.update(this);
        navModule.update(this, hwCollection);
        if (!prioritizeOdometry || updateCount % 3 == 0) {
            driveModule.update(this, hwCollection);
        }
        if (!prioritizeOdometry || updateCount % 3 == 1) {
        }
    }

    public void waitForUpdate() {
        if (!hardwareLoopThread.isAlive()) {
            update();
        } else {
            int curUpdateCount = diagnosticModule.getUpdateCount();
            while (opMode.opModeIsActive() && diagnosticModule.getUpdateCount() <= curUpdateCount) {
                Thread.yield();
            }
        }
    }

    public void waitForFullUpdate() {
        if (!hardwareLoopThread.isAlive()) {
            update();
        } else {
            int curUpdateCount = diagnosticModule.getUpdateCount();
            while (opMode.opModeIsActive() && diagnosticModule.getUpdateCount() <= curUpdateCount + 1) {
                Thread.yield();
            }
        }
    }
}
