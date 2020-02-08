package org.firstinspires.ftc.teamcode.adultbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.common.FtcGamePad;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

import java.util.ArrayList;
import java.util.Arrays;

public class MecDrive implements IDrive {

    ArrayList<DcMotorEx> motors;



    boolean isReverse = false;

    @Override
    public void AddMotors(DcMotorEx[] motors) {
        this.motors = new ArrayList<>(Arrays.asList(motors));
    }



    //private final LinearOpMode opMode;
    //private final org.firstinspires.ftc.teamcode.Robot robot;

    private static final double MIN_SPEED = 0.2;
    protected static final float mmPerInch        = 25.4f;

//    private final DcMotor fl;
//    private final DcMotor fr;
//    private final DcMotor bl;
//    private final DcMotor br;





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


    public void handle(FtcGamePad driverGamepad){
        double h, v, r;

        h = Math.pow(driverGamepad.getLeftStickX(), 3) + Math.pow(driverGamepad.getRightTrigger(), 3) - Math.pow(driverGamepad.getLeftTrigger(), 3);
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


        motors.get(0).setPower(frontRight);
        motors.get(1).setPower(backRight);
        motors.get(2).setPower(frontLeft);
        motors.get(3).setPower(backLeft);

//        fl.setPower(frontLeft);
//        fr.setPower(frontRight);
//        bl.setPower(backLeft);
//        br.setPower(backRight);
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


        motors.get(0).setPower(frontRight);
        motors.get(1).setPower(backRight);
        motors.get(2).setPower(frontLeft);
        motors.get(3).setPower(backLeft);

//        fl.setPower(frontLeft);
//        fr.setPower(frontRight);
//        bl.setPower(backLeft);
//        br.setPower(backRight);

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
