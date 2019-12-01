package org.firstinspires.ftc.teamcode.adultbot.hardware.sensors;

import org.firstinspires.ftc.teamcode.adultbot.util.ValueHistory;

public class AveragedSensor implements AnalogSensor {
    private final AnalogSensor sensor;
    private final ValueHistory valueHistory;

    /**
     * @param sensor      the sensor to average
     * @param numReadings the number of readings to average
     */
    public AveragedSensor(AnalogSensor sensor, int numReadings) {
        this.sensor = sensor;
        valueHistory = new ValueHistory(numReadings);
    }

    /**
     * @return true once the first numReadings have been read
     */
    public boolean isReady() {
        return valueHistory.areAllActive();
    }

    @Override
    public Double getValue() {
        return valueHistory.getAverage();
    }

    public double getStandardDeviation() {
        return valueHistory.getStandardDeviation();
    }

    /**
     * read the sensor and update the average
     */
    public void act() {
        valueHistory.replaceOldestValue(sensor.getValue());
    }
}
