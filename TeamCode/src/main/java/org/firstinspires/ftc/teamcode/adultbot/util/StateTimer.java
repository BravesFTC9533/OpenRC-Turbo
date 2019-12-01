package org.firstinspires.ftc.teamcode.adultbot.util;

public class StateTimer {
    private long endTime;

    /**
     * Start the timer
     *
     * @param durationMillis how long the timer will run in milliseconds
     */
    public void init(long durationMillis) {
        this.endTime = durationMillis + System.currentTimeMillis();
    }

    /**
     * @return whether or not the time has elapsed yet
     */
    public boolean isDone() {
        return System.currentTimeMillis() >= endTime;
    }
}