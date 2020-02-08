package org.firstinspires.ftc.teamcode.adultbot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.FtcGamePad;

public interface IDrive {

    void AddMotors(DcMotorEx[] motors);

    void AddIMU(BNO055IMU imu);

    

    void setIsReverse(boolean reverse);
    void toggleIsReverse();

    void drive(double ly, double lx, double rx);
    void handle(FtcGamePad driverGamepad);

    void stop();

    void setMode(DcMotor.RunMode runMode);


    void changeDriveMode();
    DriveMode getDriveMode();

}
