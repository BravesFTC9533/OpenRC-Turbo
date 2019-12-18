package org.firstinspires.ftc.teamcode.sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Sensor {

    protected final HardwareMap hardwareMap;

    public Sensor(HardwareMap hardwareMap) {this.hardwareMap = hardwareMap;}

}
