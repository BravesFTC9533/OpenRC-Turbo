package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.common.FtcGamePad;
import org.firstinspires.ftc.teamcode.common.Robot;

public class MechDrive extends Drive {

    public static final double COMPASS_CORRECT_SPEED = 0.1;
    public static final double ACCEL_CORRECT_SPEED = 0.1;

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

    public void strafe(double h, double inches, double timeoutSeconds) {
        if(inches < 0) {
            inches = -inches;
            h = -h;
        }
        double startCompass = getCompass();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        int flPositon = robot.fl.getCurrentPosition();
        double targetPosition = Robot.COUNTS_PER_INCH * Robot.MECH_COUNTS_PER_INCH * inches;

        while(opMode.opModeIsActive() && timer.seconds() < timeoutSeconds) {
            int newTickPosition = robot.fl.getCurrentPosition();
            if(Math.abs(newTickPosition - flPositon) >= (targetPosition)) break;
            double compassDiff = Range.clip(getCompass() - startCompass, -1, 1);
            compassDiff /= 10;

            drive(0, h, compassDiff);
        }
        stop();
    }

    public void strafe(double h) {
        double startCompass = getCompass();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(opMode.opModeIsActive()) {
            double compassDiff = Range.clip(getCompass() - startCompass, -1, 1);
            compassDiff /= 10;

            drive(0, h, compassDiff);
        }
        stop();
    }

    private double getCompass() {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
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

        h = Math.pow(-driverGamepad.getLeftStickX(), 3) + Math.pow(driverGamepad.getLeftTrigger(), 3)
                - Math.pow(driverGamepad.getRightTrigger(), 3);
        v = Math.pow(-driverGamepad.getLeftStickY(), 3);
        r = driverGamepad.getRightStickX();

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

        fl.setVelocity(frontLeft * Robot.MAX_NEVE_VELOCITY);
        fr.setVelocity(frontRight * Robot.MAX_NEVE_VELOCITY);
        bl.setVelocity(backLeft * Robot.MAX_NEVE_VELOCITY);
        br.setVelocity(backRight * Robot.MAX_NEVE_VELOCITY);
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
        double inchesPerDegree = Robot.WHEEL_DISTANCE_INCHES / 71.95; // Find how many inches are in a degree
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

        while(opMode.opModeIsActive() && !robot.doneMoving() && runtime.seconds() <= timeoutSeconds) {}
        stop();

        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void stop() {
        drive(0, 0, 0);
    }
}
