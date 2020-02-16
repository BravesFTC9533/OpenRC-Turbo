package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.FtcGamePad;
import org.firstinspires.ftc.teamcode.drive.Drive;

public class LiftController {

    public static final int MAX_REV_VELOCITY = 600;

    public static final int MAX_LIFT = 1325;

    public static final int POSITION_1 = 0;
    public static final int POSITION_GRAB = 200;
    public static final int POSITION_3 = 618;
    public static final int POSITION_4 = 869;
    public static final int POSITION_5 = 1112;

    public static final int MAX_SWING_POSITION = 0;
    public static final int MIN_SWING_POSITION = 1;

    public final DcMotorEx lift;
    public final Servo swing;
    public final Servo grab;

    private final LinearOpMode opMode;

    private final Drive drive;

    public LiftController(LinearOpMode opMode, HardwareMap hardwareMap, Drive drive) {
        this.opMode = opMode;
        this.drive = drive;
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        swing = hardwareMap.get(Servo.class, "swing");
        grab = hardwareMap.get(Servo.class, "grabArm");

        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        swing.setDirection(Servo.Direction.REVERSE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init() {
        grab.setPosition(1);
    }

    public void positionForBrick() {

        drive.stop();

        grab.setPosition(0.65);

        if(lift.getCurrentPosition() != POSITION_4) {
            goTo(1, POSITION_4);
        }

        swing.setPosition(0.84);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(opMode.opModeIsActive() && timer.seconds() < 1.25) {}

        grab.setPosition(0.5);
    }

    public void grabAndMoveBrick() {
        drive.stop();

        if(lift.getCurrentPosition() != POSITION_3) {
            goTo(1, POSITION_3);
        }

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(opMode.opModeIsActive() && timer.seconds() < 0.85) {}

        grab.setPosition(1);

        timer.reset();
        while(opMode.opModeIsActive() && timer.seconds() < 0.5) {}

        if(lift.getCurrentPosition() != POSITION_5) {
            goTo(1, POSITION_5);
        }

        timer.reset();
        while(opMode.opModeIsActive() && timer.seconds() < 0.75) {}

        swing.setPosition(MAX_SWING_POSITION);
    }

    public void setLiftPower(double power) {
        lift.setVelocity(MAX_REV_VELOCITY * power);
    }

    public void handle(int button, boolean pressed) {
        switch (button) {
//            case FtcGamePad.GAMEPAD_DPAD_DOWN:
//                if(pressed) {
//                    if(lift.getCurrentPosition() <= 0) {setLiftPower(0);} else
//                    setLiftPower(-1);
//                } else {
//                    setLiftPower(0);
//                }
//                break;
//            case FtcGamePad.GAMEPAD_DPAD_UP:
//                if(pressed) {
//                    if(lift.getCurrentPosition() >= MAX_LIFT) {setLiftPower(0);} else
//                    setLiftPower(1);
//                } else {
//                    setLiftPower(0);
//                }
//                break;
            case FtcGamePad.GAMEPAD_LBUMPER:
                if(pressed) {
                    positionForBrick();
                }
                break;
            case FtcGamePad.GAMEPAD_RBUMPER:
                if(pressed) {
                    grabAndMoveBrick();
                }
                break;
            case FtcGamePad.GAMEPAD_B:
                if(pressed) {
                    if (grab.getPosition() > 0.5) {
                        grab.setPosition(0.5);
                    } else {
                        grab.setPosition(1);
                    }
                }
                break;
            case FtcGamePad.GAMEPAD_Y:
                if(pressed) {
                    goTo(1, MAX_LIFT);
                }
                break;
            case FtcGamePad.GAMEPAD_DPAD_DOWN:
                if(pressed)
                    goTo(1, POSITION_1);
                break;
            case FtcGamePad.GAMEPAD_DPAD_LEFT:
                if(pressed)
                    goTo(1, POSITION_3);
                break;
            case FtcGamePad.GAMEPAD_DPAD_UP:
                if(pressed)
                    goTo(1, POSITION_4);
                break;
            case FtcGamePad.GAMEPAD_DPAD_RIGHT:
                if(pressed)
                    goTo(1, POSITION_5);
                break;
        }
    }

    public void goTo(int power, int position) {
        lift.setTargetPosition(position);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setLiftPower(power);
    }

    public void holdPosition() {
        lift.setTargetPosition(lift.getTargetPosition());
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setLiftPower(1);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
