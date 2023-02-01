package org.quixilver8404.foxtrot;

import com.google.gson.Gson;

import org.quixilver8404.powerplay.control.ActionSet;
import org.quixilver8404.powerplay.control.PurePursuitPath;
import org.quixilver8404.powerplay.control.WaypointPPPath;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class FoxtrotPath implements Cloneable {

    public static final double FIELD_SIZE = 141.0;

    private double[] xVals;
    private double[] yVals;
    private double[] headings;
    private double[] velocities;
    private int pointCount;
    private Map<Integer, ArrayList<Integer>> actions = new HashMap<>();

    public FoxtrotPath(String jsonStr, int config) {
        loadFromJsonStr(jsonStr, config);
    }

    public FoxtrotPath(String jsonStr) {
        this(jsonStr, 0);
    }

    public FoxtrotPath(InputStream jsonInputStream, int config) throws IOException {
        int size = jsonInputStream.available();
        byte[] buffer = new byte[size];
        //noinspection ResultOfMethodCallIgnored
        jsonInputStream.read(buffer);
        jsonInputStream.close();
        String jsonStr = new String(buffer, StandardCharsets.UTF_8);
        loadFromJsonStr(jsonStr, config);
    }

    public FoxtrotPath(InputStream jsonInputStream) throws IOException {
        this(jsonInputStream, 0);
    }

    private void loadFromJsonStr(String jsonStr, int config) {
        Gson gson = new Gson();
        Map obj = gson.fromJson(jsonStr, Map.class);
        Map output = (Map) obj.get("output");
        Map configEntries = (Map) output.get(String.valueOf(config));
        xVals = toPrimitiveDoubleArray(((ArrayList) configEntries.get("xVals")).toArray());
        yVals = toPrimitiveDoubleArray(((ArrayList) configEntries.get("yVals")).toArray());
        headings = toPrimitiveDoubleArray(((ArrayList) configEntries.get("headings")).toArray());
        velocities = toPrimitiveDoubleArray(((ArrayList) configEntries.get("velocities")).toArray());
        pointCount = xVals.length;

        for (Object action : (ArrayList) configEntries.get("actions")) {
            int t = ((Double) ((ArrayList) action).get(0)).intValue();
            int actionVal = ((Double) ((ArrayList) action).get(1)).intValue();
            if (!actions.containsKey(t)) {
                actions.put(t, new ArrayList<>());
            }
            actions.get(t).add(actionVal);
        }
    }

    public PurePursuitPath getPPPath() {
        return new WaypointPPPath(xVals, yVals, headings, velocities);
    }

    public ActionSet getActionSet() {
        return new ActionSet(actions);
    }

    public void flip(double reflectX) {
        for (int i = 0; i < pointCount; i++) {
            xVals[i] = 2 * reflectX - xVals[i];
            headings[i] = (3 * Math.PI - headings[i]) % (2 * Math.PI);
        }
    }

    public void flip() {
        flip(FIELD_SIZE / 2);
    }

    public double startX() {
        return xVals[0];
    }

    public double startY() {
        return yVals[0];
    }

    public double startHeading() {
        return headings[0];
    }

    public double endX() {
        return xVals[pointCount - 1];
    }

    public double endY() {
        return yVals[pointCount - 1];
    }

    public double endHeading() {
        return headings[pointCount - 1];
    }

    @Override
    public Object clone() throws CloneNotSupportedException {
        return super.clone();
    }

    private double[] toPrimitiveDoubleArray(Object[] a) {
        double[] a2 = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            a2[i] = (Double) a[i];
        }
        return a2;
    }
}