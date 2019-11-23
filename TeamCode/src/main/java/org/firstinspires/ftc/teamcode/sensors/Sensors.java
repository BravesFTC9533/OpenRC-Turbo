package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sensors {

    public static final int V3 = 3;
    public static final int V2 = 2;

    public RevColorSensorV3 colorSensor;

    public Sensors(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
    }

    public boolean isSkyStone() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        int cc = r/b * g/b;

        if(cc <= V3 && cc >= V2) return true;
        return false;
    }

}
