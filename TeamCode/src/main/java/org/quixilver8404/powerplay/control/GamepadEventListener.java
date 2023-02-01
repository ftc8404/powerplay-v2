package org.quixilver8404.powerplay.control;

import java.util.ArrayList;

public class GamepadEventListener {

    public interface ButtonEventHandler {

        boolean isPressed();

        void onPress();

        void onRelease();
    }

    public interface LinearEventHandler {
        double getValue();

        void onChange(double value);
    }

    private final ArrayList<ButtonEventHandler> buttonHandlers;
    private final ArrayList<Boolean> buttonStates;

    private final ArrayList<LinearEventHandler> linearHandlers;
    private final ArrayList<Double> linearValues;

    public void update() {
        // update button handlers
        int buttonCount = buttonHandlers.size();
        for (int i = 0; i < buttonCount; i++) {
            ButtonEventHandler buttonEventHandler = buttonHandlers.get(i);
            boolean prevState = buttonStates.get(i);
            boolean newState = buttonEventHandler.isPressed();
            if (newState) {
                if (!prevState) {
                    buttonEventHandler.onPress();
                    buttonStates.set(i, true);
                }
            } else if (prevState) {
                buttonEventHandler.onRelease();
                buttonStates.set(i, false);
            }
        }

        // update linear handlers
        int linearCount = linearHandlers.size();
        for (int i = 0; i < linearCount; i++) {
            LinearEventHandler linearEventHandler = linearHandlers.get(i);
            double prevValue = linearValues.get(i);
            double newValue = linearEventHandler.getValue();
            if (newValue != prevValue) {
                linearEventHandler.onChange(newValue);
                linearValues.set(i, newValue);
            }
        }
    }

    public GamepadEventListener() {
        buttonHandlers = new ArrayList<>();
        buttonStates = new ArrayList<>();

        linearHandlers = new ArrayList<>();
        linearValues = new ArrayList<>();
    }

    public void addButtonEventHandler(ButtonEventHandler buttonEventHandler) {
        buttonHandlers.add(buttonEventHandler);
        buttonStates.add(buttonEventHandler.isPressed());
    }

    public void removeButtonEventHandler(ButtonEventHandler buttonEventHandler) {
        int index = buttonHandlers.indexOf(buttonEventHandler);
        buttonHandlers.remove(index);
        buttonStates.remove(index);
    }

    public void addLinearEventHandler(LinearEventHandler linearEventHandler) {
        linearHandlers.add(linearEventHandler);
        linearValues.add(linearEventHandler.getValue());
    }

    public void removeLinearEventHandler(LinearEventHandler linearEventHandler) {
        int index = linearHandlers.indexOf(linearEventHandler);
        linearHandlers.remove(index);
        linearValues.remove(index);
    }

}
