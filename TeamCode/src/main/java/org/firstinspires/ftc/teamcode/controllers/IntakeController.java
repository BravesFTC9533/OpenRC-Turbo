package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.FtcGamePad;

public class IntakeController {

    public static final int MAX_REV_VELOCITY = 600;
    public static final int MAX_INTAKE_TICKS = 100;

    public final DcMotorEx intake;
    public final Servo leftIntake;
    public final Servo rightIntake;

    public final LinearOpMode opMode;

    public IntakeController(LinearOpMode opMode, HardwareMap hardwareMap) {
        this.opMode = opMode;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftIntake = hardwareMap.servo.get("leftIntake");
        rightIntake = hardwareMap.servo.get("rightIntake");

        leftIntake.setDirection(Servo.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init() {
        openIntake();
    }

    public void handle(int button, boolean pressed) {
        switch (button) {
            case FtcGamePad.GAMEPAD_X:
                if(pressed) {
                    leftIntake.setPosition(0);
                    rightIntake.setPosition(0);
                } else {
                    leftIntake.setPosition(0.5);
                    rightIntake.setPosition(0.5);
                }
                break;
//            case FtcGamePad.GAMEPAD_B:
//                if(pressed) {
//                    if(intake.getCurrentPosition() > 0) {
//                        closeIntake();
//                    } else {
//                        openIntake();
//                    }
//                }
        }
    }

    public void openIntake() {
        intake.setTargetPosition(MAX_INTAKE_TICKS);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setPower(1);
        while(opMode.opModeIsActive() && intake.isBusy()) {}
        intake.setPower(0);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void lockPosition() {
        intake.setTargetPosition(intake.getCurrentPosition());
        intake.setVelocity(MAX_REV_VELOCITY);
    }

    public void closeIntake() {
        intake.setTargetPosition(0);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setVelocity(MAX_REV_VELOCITY);
        while(opMode.opModeIsActive() && intake.isBusy()) {}
        intake.setPower(0);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
