package org.quixilver8404.powerplaycode.control.base.modules;

import static org.firstinspires.ftc.robotcore.internal.system.Misc.TAG;

import android.content.res.Resources;
import android.util.Log;

import org.apache.commons.io.IOUtils;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.quixilver8404.breakout.controller.Breakout;
import org.quixilver8404.breakout.feedforward.ActionEventListener;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.powerplaycode.control.base.Robot;
import org.quixilver8404.powerplaycode.util.Vector3;
import org.quixilver8404.simulator.MecanumKinematics;
import java.io.*;

import java.io.InputStream;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

public class BreakoutModule {

    protected Breakout[] breakoutVariants;
    protected Breakout breakout;
    final protected Config config;
    final protected List<ActionEventListener> actionEventListeners;
    public byte[] bytes;

    protected boolean isStopped;

    public BreakoutModule(final Robot robot, final int variants) {
        config = robot.config;
        actionEventListeners = robot.robotActions;
        breakout = null;
        isStopped = false;
        if (variants == 0) {
            breakoutVariants = new Breakout[1];
        } else {
            breakoutVariants = new Breakout[variants];
        }
    }

    public void update(final Robot robot) {

        if (isStopped || breakout == null) {
            if (robot.driveModule.movementMode == DriveModule.MovementMode.AUTON) {
                robot.driveModule.setPowerSettings(new double[]{0, 0, 0, 0});
            }
            return;
        }

        if (breakout.isFinished()) {
            robot.driveModule.setPowerSettings(new double[]{0, 0, 0, 0});
            return;
        }

//        breakout.setVoltage(robot.hardwareCollection.revHub1.getInputVoltage(VoltageUnit.VOLTS));

        final Vector3 pos = robot.poseModule.getPos();
        final Vector3 vel = robot.poseModule.getVel();

        final org.quixilver8404.breakout.util.Vector3 breakoutPos = new org.quixilver8404.breakout.util.Vector3(pos.x(), pos.y(), pos.theta());
        final org.quixilver8404.breakout.util.Vector3 breakoutVel = new org.quixilver8404.breakout.util.Vector3(vel.x(), vel.y(), vel.theta());

        final double[] powerSettings = breakout.iterate(breakoutPos, breakoutVel, robot.hardwareCollection.clock.getDeltaTimeMS()/1000d);
        Log.d("HI","-------"+ Arrays.toString(powerSettings));
        robot.driveModule.setPowerSettings(powerSettings);

        Log.d("Breakout", "---------last known s: " + breakout.path.getLastKnownS());
    }

    public synchronized void stop() {
        isStopped = true;
        Log.d("Breakout", "--------- true");
    }

    public synchronized void resume() {
        isStopped = false;
        Log.d("Breakout", "--------- false");
    }

    public synchronized void init(final InputStream foxtrotFile) throws IOException {
        bytes = null;
        bytes = IOUtils.toByteArray(foxtrotFile);
        Scanner s = new Scanner(new InputStreamReader(new ByteArrayInputStream(bytes)));
        while(s.hasNext()) System.out.println(s.nextLine());
        Log.d("bytes", Arrays.toString(bytes));
        System.out.println("length: " + breakoutVariants.length);
        if (breakoutVariants.length == 1) {
            breakoutVariants[0] = new Breakout(bytes, 0, actionEventListeners, config);
            Log.d("Breakout", "--------Variants length = 1");
            return;
        }
        for (int i = 0; i < breakoutVariants.length; i++) {
            Log.d("Breakout", "--------Variants length more than 1");
            breakoutVariants[i] = new Breakout(bytes, i + 1, actionEventListeners, config);
        }
    }

    public synchronized void setVariant(final int variant) {
        if (breakoutVariants.length == 1) {
            Log.d("Breakout", "--------Variants length = 1 in setVariant");
            breakout = breakoutVariants[0];
        } else if (variant == 0) {
            Log.d("Breakout", "--------Variant = 0");
            breakout = breakoutVariants[0];
        } else {
            breakout = breakoutVariants[variant - 1];
        }
    }
}