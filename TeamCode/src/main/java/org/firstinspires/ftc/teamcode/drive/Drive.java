package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.FtcGamePad;

public abstract class Drive {

    public enum TurnDirection {
        CLOCKWISE, COUNTER_CLOCKWISE
    }

    protected final LinearOpMode opMode;

    public Drive(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public abstract void drive(double v, double h, double r);
    public abstract void handleTeleop(FtcGamePad gamePad);
    public abstract void moveByInches(double power, float inches, double timeoutSeconds);
    public abstract void moveByInches(double power, float leftInches, float rightInches, double timoutSeconds);
    public abstract void turnDegrees(double power, TurnDirection turnDirection, int degrees, double timeoutSeconds);
    public abstract void encoderDrive(double power, int leftTicks, int rightTicks, double timeoutSeconds);
    public abstract void stop();

}
