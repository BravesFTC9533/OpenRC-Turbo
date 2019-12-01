package org.firstinspires.ftc.teamcode.adultbot.hardware.sensors;


public class CalibratedLineSensor implements DigitalSensor {
    private boolean ready = true;
    private final AnalogSensor lineSensor;
    private static final int NUM_CALIBRATIONS = 100;
    private int numReadings;
    private double average, deviation, threshold = 80;
    private final double readings[] = new double[NUM_CALIBRATIONS];
    private boolean seeingLine = false;
    private double value;

    /**
     * @param lineSensor the raw line sensor
     */
    public CalibratedLineSensor(AnalogSensor lineSensor) {
        this.lineSensor = lineSensor;
    }

    /**
     * start calibrating the sensor
     */
    public void calibrate() {
        ready = false;
        average = 0;
        numReadings = 0;
        deviation = 0;
    }

    /**
     * @return true when done calibrating
     */
    public boolean isReady() {
        return ready;
    }

    public double getThreshold() {
        return threshold;
    }

    /**
     * @return true if the sensor is over the line
     */
    @Override
    public Boolean getValue() {
        return seeingLine;
    }

    /**
     * read the sensor value and use it to calibrate/update the average
     */
    public void act() {
        value = lineSensor.getValue();
        if (!ready) {
            if (numReadings < NUM_CALIBRATIONS) {
                //add a value
                readings[numReadings] = value;
                numReadings++;

                //update the average
                average = (average * (numReadings - 1) + value) / (numReadings);

            } else {
                //find deviation
                for (double reading : readings) {
                    deviation += (reading - average) * (reading - average);
                }
                deviation /= Math.sqrt(numReadings);

                threshold = (average - deviation) * (average - deviation) / average * 3.5;
                ready = true;
            }
        }
        seeingLine = value >= threshold;
    }

    /**
     * @return raw line sensor
     */
    public AnalogSensor getRawLineSensor() {
        return lineSensor;
    }
}
