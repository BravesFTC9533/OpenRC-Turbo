package org.firstinspires.ftc.teamcode.adultbot.hardware.sensors;


import org.firstinspires.ftc.teamcode.adultbot.util.ImmutableList;

public class DoubleLineSensorArray extends NLineSensorArray {
    public DoubleLineSensorArray(LineSensorArray leftLineSensorArray, LineSensorArray rightLineSensorArray) {
        super(ImmutableList.of(leftLineSensorArray, rightLineSensorArray));
    }
}
