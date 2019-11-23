package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sensors {

    public static final int R = 100;
    public static final int B = 81;

    public ColorSensor colorSensor;

    public Sensors(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        colorSensor.enableLed(false);
    }

    public boolean isSkyStone() {
        int r = colorSensor.red();
        int b = colorSensor.blue();

        if(r/b <= 0) return true;
        return false;
    }

}
