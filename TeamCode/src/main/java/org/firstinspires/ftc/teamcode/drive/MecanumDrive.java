package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.common.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.common.FtcGamePad;

import java.util.ArrayList;


/**
 * Created by 9533 on 2/3/2018.
        */


public class MecanumDrive implements IDrive {

    //private final Robot robot;

    private Robot robot;

    private static final double MIN_SPEED = 0.2;
    protected static final float mmPerInch        = 25.4f;

    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;

    private LinearOpMode opMode;

    boolean reverse = false;

    public enum StraifDirection {
        LEFT, RIGHT
    }
//
//    public MecanumDrive(Robot robot, FtcGamePad driveGamepad){
//        this.driverGamepad = driveGamepad;
//        this.robot = robot;
//    }

    public MecanumDrive(LinearOpMode opMode, Robot robot) {
        this.opMode = opMode;
        this.robot = robot;
        this.fl = robot.frontLeft;
        this.fr = robot.frontRight;
        this.bl = robot.backLeft;
        this.br = robot.backRight;
    }

    public boolean getIsReverse(){
        return reverse;
    }

    public void setIsReverse(boolean value){
        reverse = value;
    }

    public void gamePadButtonEvent(FtcGamePad gamepad, int button, boolean pressed) {

    }

    public void handle(FtcGamePad driverGamepad){

        //mechDrive.Drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

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

        if(getIsReverse()) {
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

        //robot.Drive(frontLeft, frontRight, backLeft, backRight)
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
    public void drive(double left, double right) {

    }

    @Override
    public void stop() {
        drive(0, 0, 0);
    }

    @Override
    public void setMode(DcMotor.RunMode runMode) {
        fl.setMode(runMode);
        fr.setMode(runMode);
        bl.setMode(runMode);
        br.setMode(runMode);
    }

    @Override
    public void driveToPosition(int leftPosition, int rightPosition, double power) {
        addTargetPositions(robot.leftMotors, leftPosition);
        addTargetPositions(robot.rightMotors, rightPosition);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorsPowers(robot.allMotors, power);

        while(opMode.opModeIsActive() && robot.isBusy()) {}

        stop();
    }

    public void straif(StraifDirection direction, double power, float seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        if(direction == StraifDirection.RIGHT)
            power = -power;

        drive(0, power, 0);

        while(opMode.opModeIsActive() && timer.seconds() <= seconds) {}

        stop();

    }


    // Scale motor power based on the max for all wheels
    // 1, 1, 1, 3 will become .33, .33, .33, 1
    public static double scalePower(double value, double max){
        if(max == 0){return  0;}
        return  value / max;
    }

    // motor power clipping helper
    public static double clipMotorPower(double value){
        return Range.clip(value, -1, 1);
    }

    public static double scale(double power){
        int modifier = 1;

        if (power == 0 )
        {
            return 0;
        }

        if(power < 0){
            modifier *= -1;
        }

        return  (power * power * modifier);
    }

    public void addTargetPositions(ArrayList<DcMotor> motors, int ticks) {
        for(DcMotor motor : motors) {
            motor.setTargetPosition(motor.getCurrentPosition() + ticks);
        }
    }

    public void setMotorsPowers(ArrayList<DcMotor> motors, double power) {
        for(DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    public void setZeroPowerBehaviors(ArrayList<DcMotor> motors, DcMotor.ZeroPowerBehavior behavior) {
        for(DcMotor motor : motors) {
            motor.setZeroPowerBehavior(behavior);
        }
    }

    public void driveByEncoderTicks(int ticks, double power) {
        addTargetPositions(robot.allMotors, ticks);
        setMotorsPowers(robot.allMotors, power);
    }

    public boolean atPosition(ArrayList<DcMotor> motors, int expectedPosition) {
        for(DcMotor motor : motors) {
            if(motor.getCurrentPosition() != expectedPosition) {
                return false;
            }
        }
        return true;
    }

    public void moveByInches(double power, double inches, double timeoutSeconds) {
        moveByInches(power, inches, inches, timeoutSeconds);
    }

    public void moveByInches(double power, double leftInches, double rightInches, double timeoutSeconds) {
        encoderDrive(power, (int) Robot.COUNTS_PER_INCH * leftInches,
                (int) Robot.COUNTS_PER_INCH * rightInches, timeoutSeconds);
    }

    public void moveByMillimeters(int millimeters, double power, double timeoutSeconds) {
        moveByInches(millimeters / mmPerInch, power, timeoutSeconds);
    }

    public void turnDegrees(TurnDirection direction, int degrees, double power, double timeoutSeconds) {
        double inchesPerDegree = Robot.WHEEL_DISTANCE_INCHES / 90; // Find how many inches are in a degree
        degrees *= inchesPerDegree;

        if(direction == TurnDirection.COUNTER_CLOCKWISE) {
            degrees = -degrees;
        }

        moveByInches(power, degrees, -degrees, timeoutSeconds);
    }

    public void encoderDrive(double targetSpeed, double leftTicks, double rightTicks, double timeoutSeconds) {
        addTargetPositions(robot.leftMotors, (int) leftTicks);
        addTargetPositions(robot.rightMotors, (int) rightTicks);

        robot.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorsPowers(robot.allMotors, targetSpeed);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(opMode.opModeIsActive() && robot.isBusy() && timer.seconds() <= timeoutSeconds) {}

        robot.stop();
        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}