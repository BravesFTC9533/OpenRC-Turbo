package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.FtcGamePad;

public class IntakeController {

    private static final int MAX_REV_VELOCITY = 600;
    private static final int MAX_INTAKE_TICKS = 100;

    public final DcMotorEx intake;
    private final Servo leftIntake;
    private final Servo rightIntake;

    private final LinearOpMode opMode;

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

    private boolean yAlreadyPressed = false;
    public void handle(int button, boolean pressed) {
        switch (button) {
            case FtcGamePad.GAMEPAD_X:
                if(pressed) {
                    if(leftIntake.getPosition() > 0) {
                        intakeIn();
                    } else {
                        stopIntake();
                    }
                }
                break;
            case FtcGamePad.GAMEPAD_A:
                if(pressed) {
                    if(leftIntake.getPosition() < 1) {
                        intakeOut();
                    } else {
                        stopIntake();
                    }
                }
                break;
            case FtcGamePad.GAMEPAD_BACK:
                if(pressed) {
                    if(!yAlreadyPressed) {
                        init();
                        yAlreadyPressed = true;
                    }
                }
                break;
        }
    }

    private void openIntake() {
        intake.setTargetPosition(MAX_INTAKE_TICKS);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setPower(1);
        while(opMode.opModeIsActive() && intake.isBusy()) {}
        intake.setPower(0);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void intakeIn() {
        leftIntake.setPosition(0);
        rightIntake.setPosition(0);
    }

    private void intakeOut() {
        leftIntake.setPosition(1);
        rightIntake.setPosition(1);
    }

    private void stopIntake() {
        leftIntake.setPosition(0.5);
        rightIntake.setPosition(0.5);
    }

    @Deprecated
    public void closeIntake() {
        intake.setTargetPosition(0);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setVelocity(MAX_REV_VELOCITY);
        while(opMode.opModeIsActive() && intake.isBusy()) {}
        intake.setPower(0);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
