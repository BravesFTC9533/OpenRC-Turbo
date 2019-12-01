package org.firstinspires.ftc.teamcode.adultbot.hardware.sensors;

import org.firstinspires.ftc.teamcode.adultbot.util.units.Distance;


public interface DistanceSensor extends AnalogSensor {
    /**
     * @return the distance that the sensor is reading
     */
    Distance getDistance();
}
