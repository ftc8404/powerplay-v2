package org.quixilver8404.powerplay.control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.quixilver8404.powerplay.util.MovingAverageFilter;
import org.quixilver8404.powerplay.util.Vector3;
import org.quixilver8404.powerplay.util.measurement.Pose2D;

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
    public final SlidesModule slidesModule;
    public final SusanModule susanModule;
    public final ClawModule clawModule;
    public final MSonicModule mSonicModule;
    public final CVTasksModule cvTasksModule;
    public final PositionTrackingModule poseModule;
    public final PIDPositionEstimation pidPositionEstimation;
    public final Actions actions;
    public final MovingAverageFilter movingAverageFilter;

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
        slidesModule = new SlidesModule();
        susanModule = new SusanModule();
        clawModule = new ClawModule();
        cvTasksModule = new CVTasksModule(hwCollection);
        poseModule = new PositionTrackingModule(new Vector3());//new Vector3(75 * 0.0254,(141-26) *0.0254,0));
        pidPositionEstimation = new PIDPositionEstimation(this, new Vector3());
        actions = new Actions(this);
        mSonicModule = new MSonicModule(this);
        movingAverageFilter = new MovingAverageFilter(this);


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

        criticalUpdate();

        if (prioritizeOdometry) {
            // then make other modules update less often
            if (updateCount % 3 == 0) {
                nonCriticalUpdate1();
            } else if (updateCount % 3 == 1) {
                nonCriticalUpdate2();
            }
        } else {
            nonCriticalUpdate1();
            nonCriticalUpdate2();
        }
    }

    private void criticalUpdate() {
        diagnosticModule.update(this, hwCollection);
        poseModule.update(this);
        actions.update();
        pidPositionEstimation.update();
//        taskModule.update(this, hwCollection);
//        pathFollowModule.update(this);
//        headingLockModule.update(this);
//        navModule.update(this, hwCollection);

    }

    private void nonCriticalUpdate1() {
        driveModule.update(this, hwCollection);
        clawModule.update(hwCollection);
    }

    private void nonCriticalUpdate2() {
        slidesModule.update(susanModule, hwCollection);
        susanModule.update(slidesModule, hwCollection);
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

    public void stopDriveMotors() {
        // TODO fix because this doesn't stop the robot when auton doesn't have enough time
        hwCollection.driveMotorFL.setPower(0);
        hwCollection.driveMotorFR.setPower(0);
        hwCollection.driveMotorBL.setPower(0);
        hwCollection.driveMotorBR.setPower(0);
    }
}
