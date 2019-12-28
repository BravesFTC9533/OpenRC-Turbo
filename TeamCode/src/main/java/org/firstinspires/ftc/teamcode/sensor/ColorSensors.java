package org.firstinspires.ftc.teamcode.sensor;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.sensor.ColorSensors.SensorSide.FRONT;

public class ColorSensors extends Sensor {

    public static final int SKY_STONE_CONDITION_NUMBER_MAX = 270;

    public final DistanceSensor frontDistance;
    public final ColorSensor frontColor;

    public final DistanceSensor backDistance;
    public final ColorSensor backColor;

    public enum SensorSide {
        FRONT, BACK
    }

    public ColorSensors(HardwareMap hardwareMap) {
        super(hardwareMap);

        frontDistance = hardwareMap.get(DistanceSensor.class, "front_sensor");
        frontColor = hardwareMap.get(ColorSensor.class, "front_sensor");

        backDistance = hardwareMap.get(DistanceSensor.class, "back_sensor");
        backColor = hardwareMap.get(ColorSensor.class, "back_sensor");
    }

    public double getSensorDistance(SensorSide sensorSide, DistanceUnit distanceUnit) {
        if(sensorSide == FRONT) {
            return frontDistance.getDistance(distanceUnit);
        } else if(sensorSide == SensorSide.BACK) {
            return backDistance.getDistance(distanceUnit);
        }
        return frontDistance.getDistance(distanceUnit);
    }

    public int getR(SensorSide sensorSide) {
        switch (sensorSide) {
            case FRONT:
                return frontColor.red();
            case BACK:
                return backColor.red();
        }
        return frontColor.red();
    }

    public int getB(SensorSide sensorSide) {
        switch (sensorSide) {
            case FRONT:
                return frontColor.blue();
            case BACK:
                return backColor.blue();
        }
        return frontColor.blue();
    }

    public int getG(SensorSide sensorSide) {
        switch (sensorSide) {
            case FRONT:
                return frontColor.green();
            case BACK:
                return backColor.green();
        }
        return frontColor.green();
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
