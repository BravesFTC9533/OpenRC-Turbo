package org.firstinspires.ftc.teamcode.adultbot.hardware.sensors;

public interface Gyro {
    double getHeading();
    //    void update();
    boolean isCalibrating();
    void stop();

    void setActive(boolean isActive);
}