package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.FtcGamePad;
import org.firstinspires.ftc.teamcode.common.Robot;

public class MechDrive extends Drive {

    private static final double MIN_SPEED = 0.2;

    public final Robot robot;

    public final DcMotorEx fl;
    public final DcMotorEx fr;
    public final DcMotorEx bl;
    public final DcMotorEx br;

    public enum StrafeDirection {
        LEFT, RIGHT
    }

    public MechDrive(Robot robot, LinearOpMode opMode) {
        super(opMode);
        this.robot = robot;
        this.fl = robot.fl;
        this.fr = robot.fr;
        this.bl = robot.bl;
        this.br = robot.br;
    }

    // Scale motor power based on the max for all wheels
    // 1, 1, 1, 3 will become .33, .33, .33, 1
    public static double scalePower(double value, double max){
        if(max == 0){return  0;}
        return  value / max;
    }

    public static double clipMotorPower(double value){
        return Range.clip(value, -1, 1);
    }

    public void addTargetPosition(int leftTicks, int rightTicks) {
        fl.setTargetPosition(fl.getCurrentPosition() + leftTicks);
        bl.setTargetPosition(bl.getCurrentPosition() + leftTicks);
        fr.setTargetPosition(fr.getCurrentPosition() + rightTicks);
        br.setTargetPosition(br.getCurrentPosition() + rightTicks);
    }

    @Override
    public void drive(double v, double h, double r) {
        // add vectors
        double frontLeft =  v-h+r;
        double frontRight = v+h-r;
        double backRight =  v-h-r;
        double backLeft =   v+h+r;

        // since adding vectors can go over 1, figure out max to scale other wheels
        double max = Math.max(
                Math.abs(backLeft),
                Math.max(
                        Math.abs(backRight),
                        Math.max(
                                Math.abs(frontLeft), Math.abs(frontRight)
                        )
                )
        );
        // only need to scale power if max > 1
        if(max > 1){
            frontLeft = scalePower(frontLeft, max);
            frontRight = scalePower(frontRight, max);
            backLeft = scalePower(backLeft, max);
            backRight = scalePower(backRight, max);
        }

        fl.setPower(frontLeft);
        fr.setPower(frontRight);
        bl.setPower(backLeft);
        br.setPower(backRight);
    }

    @Override
    public void handleTeleop(FtcGamePad driverGamepad){
        double h, v, r;

        h = -driverGamepad.getLeftStickX();
        v = -driverGamepad.getLeftStickY();
        r = -driverGamepad.getRightStickX();

        if(Math.abs(h) < MIN_SPEED) {
            h = 0;
        }
        if(Math.abs(v) < MIN_SPEED) {
            v = 0;
        }
        if(Math.abs(r) < MIN_SPEED){
            r = 0;
        }

        if(robot.isReverse()) {
            h *= -1;
            v *= -1;
        }


        h = clipMotorPower(h);
        v = clipMotorPower(v);
        r = clipMotorPower(r);

        // add vectors
        double frontLeft =  v-h+r;
        double frontRight = v+h-r;
        double backRight =  v-h-r;
        double backLeft =   v+h+r;

        // since adding vectors can go over 1, figure out max to scale other wheels
        double max = Math.max(
                Math.abs(backLeft),
                Math.max(
                        Math.abs(backRight),
                        Math.max(
                                Math.abs(frontLeft), Math.abs(frontRight)
                        )
                )
        );
        // only need to scale power if max > 1
        if(max > 1){
            frontLeft = scalePower(frontLeft, max);
            frontRight = scalePower(frontRight, max);
            backLeft = scalePower(backLeft, max);
            backRight = scalePower(backRight, max);
        }

        fl.setPower(frontLeft);
        fr.setPower(frontRight);
        bl.setPower(backLeft);
        br.setPower(backRight);
    }

    public void strafeSeconds(double power, StrafeDirection direction, double seconds) {
        ElapsedTime timer = new ElapsedTime();

        if(direction == StrafeDirection.RIGHT) {
            power = -power;
        }

        timer.reset();
        drive(0, power, 0);
        while(timer.seconds() < seconds) {}
        stop();
    }

    @Override
    public void moveByInches(double power, float inches, double timeoutSeconds) {
        moveByInches(power, inches, inches, timeoutSeconds);
    }

    @Override
    public void moveByInches(double power, float leftInches, float rightInches, double timeoutSeconds) {
        encoderDrive(power, (int) (Robot.COUNTS_PER_INCH * leftInches), (int) (Robot.COUNTS_PER_INCH * rightInches), timeoutSeconds);
    }

    @Override
    public void turnDegrees(double power, TurnDirection turnDirection, int degrees, double timeoutSeconds) {
        double inchesPerDegree = Robot.WHEEL_DISTANCE_INCHES / 90; // Find how many inches are in a degree
        degrees *= inchesPerDegree;

        if(turnDirection == TurnDirection.COUNTER_CLOCKWISE) {
            degrees = -degrees;
        }

        moveByInches(power, degrees, -degrees, timeoutSeconds);
    }

    @Override
    public void encoderDrive(double power, int leftTicks, int rightTicks, double timeoutSeconds) {
        addTargetPosition(leftTicks, rightTicks);
        robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(power, 0, 0);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while(opMode.opModeIsActive() && robot.isBusy() && runtime.seconds() <= timeoutSeconds) {}

        stop();
    }

    @Override
    public void stop() {
        drive(0, 0, 0);
    }
}
