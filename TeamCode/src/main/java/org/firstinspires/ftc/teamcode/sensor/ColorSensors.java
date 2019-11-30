package org.firstinspires.ftc.teamcode.sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;

public class ColorSensors extends Sensor {

    public static final int CONDITION_NUMBER = 2;

    private Robot robot;

    public enum SensorSide {
        LEFT, RIGHT
    }

    public ColorSensors(Robot robot, LinearOpMode opMode) {
        super(opMode);
        this.robot = robot;
    }

    public boolean isSkyStone(SensorSide sensorSide) {
        int r = 0, g = 0, b = 0;
        switch (sensorSide) {
            case LEFT:
                r = robot.colorSensorLeft.red();
                g = robot.colorSensorLeft.green();
                b = robot.colorSensorLeft.blue();
                break;
            case RIGHT:
                r = robot.colorSensorRight.red();
                g = robot.colorSensorRight.green();
                b = robot.colorSensorRight.blue();
                break;
        }
        int cc = (r / b) * (g / b);

        if(cc < CONDITION_NUMBER) {
            return true;
        } else {
            return false;
        }
    }

}
