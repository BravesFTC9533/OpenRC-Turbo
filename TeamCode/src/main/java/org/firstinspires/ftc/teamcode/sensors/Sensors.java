package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sensors {

    public ColorSensor colorSensor;

    public Sensors(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.colorSensor.get("color");
    }

}
