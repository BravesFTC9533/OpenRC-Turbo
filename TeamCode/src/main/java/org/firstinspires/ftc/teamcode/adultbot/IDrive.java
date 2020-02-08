package org.firstinspires.ftc.teamcode.adultbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.FtcGamePad;

public interface IDrive {

    void AddMotors(DcMotorEx[] motors);

    void setIsReverse(boolean reverse);

    void drive(double ly, double lx, double rx);


    void stop();

    void setMode(DcMotor.RunMode runMode);


}
