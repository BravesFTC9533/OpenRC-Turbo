package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.common.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.common.FtcGamePad;

import java.util.ArrayList;

/**
 * Created by 9533 on 2/3/2018.
 */

public interface IDrive {

    enum TurnDirection {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }

    boolean getIsReverse();
    void setIsReverse(boolean value);
    void handle(FtcGamePad driverGamepad);

    void drive(double ly, double lx, double rx);
    void drive(double left, double right);

    void gamePadButtonEvent(FtcGamePad gamepad, int button, boolean pressed);

    void driveByEncoderTicks(int ticks, double power);

    void moveByInches(double power, double inches, double timeoutSeconds);
    void moveByInches(double power, double leftInches, double rightInches, double timeoutSeconds);

    void turnDegrees(TurnDirection turnDirection, int degrees, double power, double timeoutSeconds);

    void stop();

    void setMode(DcMotor.RunMode runMode);

    void driveToPosition(int leftPosition, int rightPosition, double power);
}





