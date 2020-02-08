package org.firstinspires.ftc.teamcode.adultbot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.common.FtcGamePad;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

import java.util.ArrayList;
import java.util.Arrays;

public class MecDrive implements IDrive {

    ArrayList<DcMotorEx> motors;





    BNO055IMU imu;
    boolean isReverse = false;

    @Override
    public void AddMotors(DcMotorEx[] motors) {
        this.motors = new ArrayList<>(Arrays.asList(motors));
    }

    @Override
    public void AddIMU(BNO055IMU imu) {
        this.imu = imu;
    }



    //private final LinearOpMode opMode;
    //private final org.firstinspires.ftc.teamcode.Robot robot;

    private static final double MIN_SPEED = 0.2;
    protected static final float mmPerInch        = 25.4f;

//    private final DcMotor fl;
//    private final DcMotor fr;
//    private final DcMotor bl;
//    private final DcMotor br;


    private DriveMode driveMode = DriveMode.RobotOriented;



    public enum StrafeDirection {
        LEFT, RIGHT
    }


//    public MecDrive(LinearOpMode opMode, DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
//        this.opMode = opMode;
//        this.fl = fl;
//        this.fr = fr;
//        this.bl = bl;
//        this.br = br;
//
//        robot = null;
//
//    }
//
//    public MecDrive(LinearOpMode opMode, org.firstinspires.ftc.teamcode.Robot robot) {
//        this.opMode = opMode;
//        this.robot = robot;
//        this.fl = robot.frontLeft;
//        this.fr = robot.frontRight;
//        this.bl = robot.backLeft;
//        this.br = robot.backRight;
//    }

    public boolean getIsReverse(){
        return this.isReverse;

    }

    public void setIsReverse(boolean value){
        this.isReverse = value;
    }

    @Override
    public void toggleIsReverse() {
        this.isReverse = !this.isReverse;
    }


    @Override
    public void handle(FtcGamePad driverGamepad){
        double h, v, r;

        h = Math.pow(driverGamepad.getLeftStickX(), 3) + Math.pow(driverGamepad.getRightTrigger(), 3) - Math.pow(driverGamepad.getLeftTrigger(), 3);
        h = -h;
        v = Math.pow(driverGamepad.getLeftStickY(), 3) ;
        r = Math.pow(-driverGamepad.getRightStickX(), 3);

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

        if(driveMode == DriveMode.RobotOriented) {
            robotOrientedDrive(v, h, r);
        } else {
            fieldOrientedDrive(v, h, r);
        }


    }

    private void robotOrientedDrive(double v, double h, double r) {
        double rp = Math.hypot(h, v);

        double robotAngle = Math.atan2(v, h) - Math.PI / 4;

        final double front_left = rp * Math.cos(robotAngle) + r;
        final double front_right = rp * Math.sin(robotAngle) - r;
        final double rear_left = rp * Math.sin(robotAngle) + r;
        final double rear_right = rp * Math.cos(robotAngle) - r;

        setPower(front_right, rear_right, front_left, rear_left);

    }
    private void fieldOrientedDrive(double v, double h, double r) {
        Orientation angles;
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);


        if(Math.abs(v) < MIN_SPEED) { v = 0; }
        if(Math.abs(h) < MIN_SPEED) { h = 0; }
        if(Math.abs(r) < MIN_SPEED) { r = 0; }

        double theta = angles.firstAngle;

        double temp = v*Math.cos(theta) - h*Math.sin(theta);
        h = v*Math.sin(theta) + h*Math.cos(theta);
        v = temp;


        double front_left = v + r + h;
        double front_right = v - r - h;
        double rear_left = v + r - h;
        double rear_right = v - r + h;

        double max = Math.abs(front_left);
        if(Math.abs(front_right) > max) max = Math.abs(front_right);
        if(Math.abs(rear_left) > max) max = Math.abs(rear_left);
        if(Math.abs(rear_right) > max) max = Math.abs(rear_right);

        if(max > 1) {
            front_left/=max;
            front_right/=max;
            rear_left/=max;
            rear_right/=max;
        }

        setPower(front_right, rear_right, front_left, rear_left);

    }

    private void setPower(double fr, double br, double fl, double bl) {
        motors.get(0).setPower(fr);
        motors.get(1).setPower(br);
        motors.get(2).setPower(fl);
        motors.get(3).setPower(bl);
    }

    @Override
    public void drive(double v, double h, double r) {

        h = clipMotorPower(h);
        v = clipMotorPower(v);
        r = clipMotorPower(r);


        if(getIsReverse()) {
            h *= -1;
            v *= -1;
        }

        if(driveMode == DriveMode.RobotOriented) {
            robotOrientedDrive(v, h, r);
        } else {
            fieldOrientedDrive(v, h, r);
        }
    }



    @Override
    public void stop() {
        drive(0, 0, 0);
    }

    @Override
    public void setMode(DcMotor.RunMode runMode) {

        for(int x = 0;x<4;x++){
            motors.get(x).setMode(runMode);
        }
    }

    @Override
    public void changeDriveMode() {

        if(driveMode == DriveMode.RobotOriented ) {
            driveMode = DriveMode.FieldOriented;
        } else {
            driveMode = DriveMode.RobotOriented;
        }
    }

    @Override
    public DriveMode getDriveMode() {
        return  driveMode;
    }


//    @Override
//    public void drive(double ly, double lx, double rx) {
//        if(this.driveMode == DriveModes.FIELD) {
//            fieldOrientedDrive(ly, lx, rx);
//        } else {
//            robotOrientedDrive(ly, lx, rx);
//        }
//    }
//

//    private void robotOrientedDrive(double leftY, double leftX, double rightX) {
//        double r = Math.hypot(leftX, leftY);
//
//        double robotAngle = Math.atan2(leftY, leftX) - Math.PI / 4;
//
//        final double v1 = r * Math.cos(robotAngle) + rightX;
//        final double v2 = r * Math.sin(robotAngle) - rightX;
//        final double v3 = r * Math.sin(robotAngle) + rightX;
//        final double v4 = r * Math.cos(robotAngle) - rightX;
//
//        robot.Drive(v1, v2, v3, v4);
//    }
//
//    public void handle_robotOrientedDrive() {
//
//        double leftX = driverGamepad.getLeftStickX();
//        double leftY = driverGamepad.getLeftStickY();
//
//
//        if(reverse) {
//            leftX *= -1;
//            leftY *= -1;
//        }
//
//        double rightX = driverGamepad.getRightStickX();
//
//
//
//    }
//
//
//
//    public void handle_fieldOrientedDrive() {
//
//        double forward = driverGamepad.getLeftStickY();
//        double right = driverGamepad.getLeftStickX();
//        double clockwise = driverGamepad.getRightStickX();
//
//        fieldOrientedDrive(forward, right, clockwise);
//
//    }



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





}
