package org.firstinspires.ftc.teamcode.adultbot;

public enum DriveMode {
    RobotOriented("Robot"),
    FieldOriented("Field");

    private String name;
    private DriveMode(String name) {
        this.name = name;
    }

    @Override
    public String toString(){
        return name;
    }
}