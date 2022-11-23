package org.quixilver8404.powerplaycode.control.base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.powerplaycode.control.base.modules.AutoPilotModule;
import org.quixilver8404.powerplaycode.control.base.modules.BreakoutModule;
//import org.quixilver8404.powerplaycode.control.base.modules.CVTasksModule;
import org.quixilver8404.powerplaycode.control.base.modules.DriveModule;
import org.quixilver8404.powerplaycode.control.base.modules.ClawModule;
import org.quixilver8404.powerplaycode.control.base.modules.PositionTrackingModule;
import org.quixilver8404.powerplaycode.control.base.modules.SlideModule;
import org.quixilver8404.powerplaycode.control.base.modules.SusanModule;
import org.quixilver8404.powerplaycode.util.Vector3;
import org.quixilver8404.simulator.MecanumKinematics;


public class Robot {
    public final OpMode opMode;
    public final Vector3 startPos;
    public final DriveModule driveModule;
    public final AutoPilotModule autoPilotModule;
    public final BreakoutModule breakoutModule;
    public final ClawModule clawModule;
    public final SusanModule susanModule;
    public final SlideModule slideModule;
//    public final CVTasksModule cvTasksModule;
    public final PositionTrackingModule poseModule;
    public final HardwareCollection hardwareCollection;
    public final HardwareLoopThread hardwareLoopThread;
    public final Config config;
    public final RobotActions robotActions;

    public static final int MODULE_UPDATE_INTERVAL = 2;
    protected long updateCount;

    public Robot(final Vector3 startPos, final LinearOpMode opMode){
        this.opMode = opMode;
        this.startPos = startPos;
        poseModule = new PositionTrackingModule(startPos);
        hardwareCollection = new HardwareCollection(opMode.hardwareMap);
        hardwareLoopThread = new HardwareLoopThread(this, opMode);
        config = new Config(
                12, hardwareCollection.revHub1.getInputVoltage(VoltageUnit.VOLTS), 20.9345794393-10, -12,
                1.1775,0.05, 2.1, 31.4, 10.7, 0.075/2, MecanumKinematics.FindMomentOfInertia(0.323, 0.445, 10.7),
                ((0.323/2) - 0.0375), ((0.445/2) - 0.05031), 0.95, 0.95, Math.PI/2.5, 0.074418605, 0.06, 10.7/4, 10.7/4, 10.7/4, 10.7/4
        );
        robotActions = new RobotActions(this);
        driveModule = new DriveModule();
        clawModule = new ClawModule();
        susanModule = new SusanModule();
        slideModule = new SlideModule(hardwareCollection);
//        cvTasksModule = new CVTasksModule();
        breakoutModule = new BreakoutModule(this, 0);
        autoPilotModule = new AutoPilotModule(this);
        updateCount = 0;
    }
    public Robot(final Vector3 startPos, final OpMode opMode){
        this.opMode = opMode;
        this.startPos = startPos;
        poseModule = new PositionTrackingModule(startPos);
        hardwareCollection = new HardwareCollection(opMode.hardwareMap);
        hardwareLoopThread = new HardwareLoopThread(this, opMode);
        config = new Config(
                12, hardwareCollection.revHub1.getInputVoltage(VoltageUnit.VOLTS), 20.9345794393-10, -12,
                1.1775,0.05, 2.1, 31.4, 10.7, 0.075/2, MecanumKinematics.FindMomentOfInertia(0.323, 0.445, 10.7),
                ((0.323/2) - 0.0375), ((0.445/2) - 0.05031), 0.95, 0.95, Math.PI/2.5, 0.074418605, 0.06, 10.7/4, 10.7/4, 10.7/4, 10.7/4
        );
        robotActions = new RobotActions(this);
        driveModule = new DriveModule();
        clawModule = new ClawModule();
        susanModule = new SusanModule();
        slideModule = new SlideModule(hardwareCollection);
//        cvTasksModule = new CVTasksModule();
        breakoutModule = new BreakoutModule(this, 0);
        autoPilotModule = new AutoPilotModule(this);
        updateCount = 0;
    }
    public void update() {
        hardwareCollection.refreshBulkData();
        poseModule.update(this);
        breakoutModule.update(this);
        if (updateCount % MODULE_UPDATE_INTERVAL == 0) {
            driveModule.update(this, hardwareCollection);
            slideModule.update();
            susanModule.update(hardwareCollection);
            clawModule.update(hardwareCollection);
//            cvTasksModule.update(this, hardwareCollection);
        }

        updateCount++;
    }
    public void startHardwareLoop() {
        if (!hardwareLoopThread.isAlive()) {
            update();
            hardwareLoopThread.start();
        }
    }
    public void stopHardwareLoop() throws InterruptedException {
        if (hardwareLoopThread.isAlive()) {
            hardwareLoopThread.interrupt();
            hardwareLoopThread.join();
        }
    }


}
