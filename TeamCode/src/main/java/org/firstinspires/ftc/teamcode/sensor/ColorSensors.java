package org.firstinspires.ftc.teamcode.sensor;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensors extends Sensor {

    public static final int SKY_STONE_CONDITION_NUMBER_MAX = 80;

    public final DistanceSensor leftDistance;
    public final ColorSensor leftColor;

    public final DistanceSensor rightDistance;
    public final ColorSensor rightColor;

    public enum SensorSide {
        LEFT, RIGHT
    }

    public ColorSensors(HardwareMap hardwareMap) {
        super(hardwareMap);

        leftDistance = hardwareMap.get(DistanceSensor.class, "left_sensor");
        leftColor = hardwareMap.get(ColorSensor.class, "left_sensor");

        rightDistance = hardwareMap.get(DistanceSensor.class, "right_sensor");
        rightColor = hardwareMap.get(ColorSensor.class, "right_sensor");
    }

    public double getSensorDistance(SensorSide sensorSide, DistanceUnit distanceUnit) {
        if(sensorSide == SensorSide.LEFT) {
            return leftDistance.getDistance(distanceUnit);
        } else if(sensorSide == SensorSide.RIGHT) {
            return rightDistance.getDistance(distanceUnit);
        }
        return leftDistance.getDistance(distanceUnit);
    }

    public int getR(SensorSide sensorSide) {
        switch (sensorSide) {
            case LEFT:
                return leftColor.red();
            case RIGHT:
                return rightColor.red();
        }
        return leftColor.red();
    }

    public int getB(SensorSide sensorSide) {
        switch (sensorSide) {
            case LEFT:
                return leftColor.blue();
            case RIGHT:
                return rightColor.blue();
        }
        return leftColor.blue();
    }

    public int getG(SensorSide sensorSide) {
        switch (sensorSide) {
            case LEFT:
                return leftColor.green();
            case RIGHT:
                return rightColor.green();
        }
        return leftColor.green();
    }

    public int getCondition(SensorSide sensorSide) {
        int r = getR(sensorSide);
        int g = getG(sensorSide);
        int b = getB(sensorSide);
        return (r * g) / (b ^ 2);
    }

    public boolean isSkystone(SensorSide sensorSide) {
        int r = getR(sensorSide);
        int g = getG(sensorSide);
        int b = getB(sensorSide);
        int condition = (r * g) / (b ^ 2);
        if(condition <= SKY_STONE_CONDITION_NUMBER_MAX) return true;
        else return false;
    }

}
