package org.quixilver8404.energize.control;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.TreeMap;

public class ActionSet {
    private final int[] tVals;
    private final ArrayList<Integer>[] actionVals;
    private final Map<Integer, Integer> tValOfActions; // {actionID: tVal, ...}

    private int pendingIndex = 0;

    /**
     * Defaults to blue side
     */
    public ActionSet(int[] tVals, ArrayList<Integer>[] actionVals) {
        this.tVals = tVals;
        this.actionVals = actionVals;
        tValOfActions = new HashMap<>();
        computeTValOfActions();
    }

    @SuppressWarnings("unchecked")
    public ActionSet(Map<Integer, ArrayList<Integer>> actions) {
        tVals = new int[actions.size()];
        actionVals = new ArrayList[actions.size()];
        TreeMap<Integer, ArrayList<Integer>> sortedActions = new TreeMap<>(actions);
        int index = 0;
        for (int tVal : sortedActions.keySet()) {
            ArrayList<Integer> curActionVals = sortedActions.get(tVal);
            tVals[index] = tVal;
            actionVals[index] = new ArrayList<>();
            actionVals[index].addAll(Objects.requireNonNull(curActionVals));
            index++;
        }
        tValOfActions = new HashMap<>();
        computeTValOfActions();
    }

    private void computeTValOfActions() {
        for (int i = 0; i < actionVals.length; i++) {
            ArrayList<Integer> actionValList = actionVals[i];
            int tVal = tVals[i];
            for (int actionVal : actionValList) {
                tValOfActions.put(actionVal, tVal);
            }
        }
    }

    public void runUpTo(double t, BaseRobot baseRobot) {
        while (true) {
            if (pendingIndex >= tVals.length) {
                break;
            }
            int curT = tVals[pendingIndex];
            if (curT > t) {
                break;
            }
            for (Object actionVal : actionVals[pendingIndex]) {
                int actionID = (Integer) actionVal;
                AutonActions.runAction(actionID, baseRobot);
            }
            pendingIndex++;
        }
    }

    public void runRemaining(BaseRobot baseRobot) {
        runUpTo(Integer.MAX_VALUE, baseRobot);
    }

    public void reset() {
        pendingIndex = 0;
    }

    public int getTValOfAction(int actionVal) {
        if (tValOfActions.containsKey(actionVal)) {
            //noinspection ConstantConditions
            return tValOfActions.get(actionVal);
        } else {
            return 0;
        }
    }
}
