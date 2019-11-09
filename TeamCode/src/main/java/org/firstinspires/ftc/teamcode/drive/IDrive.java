package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by 9533 on 2/3/2018.
 */

public interface IDrive {


    boolean getIsReverse();
    void setIsReverse(boolean value);
    void handle();

    void drive(double ly, double lx, double rx);
    void drive(double left, double right);

    void stop();

    void setMode(DcMotor.RunMode runMode);

    void driveToPosition(int leftPosition, int rightPosition);
}





