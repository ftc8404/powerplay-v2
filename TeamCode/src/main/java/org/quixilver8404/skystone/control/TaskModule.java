package org.quixilver8404.skystone.control;

import java.util.ArrayList;

public class TaskModule {

    public static abstract class Task {
        /**
         * @param runningTimeMillis the time in milliseconds since this task began executing
         * @return true if the task is completed, false otherwise
         */
        public abstract boolean loop(int runningTimeMillis, BaseRobot baseRobot);
    }

    private ArrayList<Task> tasks;
    private ArrayList<Integer> taskStartTimesMillis;

    public TaskModule() {
        tasks = new ArrayList<>();
        taskStartTimesMillis = new ArrayList<>();
    }

    public synchronized void update(BaseRobot baseRobot, HardwareCollection hwCollection) {
        int curRunningTimeMillis = hwCollection.clock.getRunningTimeMillis();
        for (int i = 0; i < tasks.size(); ) {
            if (taskStartTimesMillis.get(i) == -1) {
                taskStartTimesMillis.set(i, curRunningTimeMillis);
            }
            boolean isTaskComplete = tasks.get(i).loop(curRunningTimeMillis - taskStartTimesMillis.get(i), baseRobot);
            if (isTaskComplete) {
                tasks.remove(i);
                taskStartTimesMillis.remove(i);
            } else {
                i++;
            }
        }
    }

    public synchronized void addTask(Task task) {
        tasks.add(task);
        taskStartTimesMillis.add(-1);
    }

    public synchronized void removeTask(Task task) {
        int removeIndex = tasks.indexOf(task);
        tasks.remove(removeIndex);
        taskStartTimesMillis.remove(removeIndex);
    }

    public synchronized void removeAllTasks() {
        tasks.clear();
        taskStartTimesMillis.clear();
    }
}
