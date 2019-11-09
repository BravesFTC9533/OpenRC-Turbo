package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.BaseController;
import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.common.FtcGamePad;

public class LiftController extends BaseController {

    public static double MAX_LIFT_HEIGHT_IN_TICKS = 1000.0;
    public static double LIFT_SPEED = 1.0;

    public DcMotor lift;
    private Telemetry telemetry;

    public Servo dragServo;
    public Servo leftLiftServo;
    public Servo rightLiftServo;

    private boolean isClosed = true;
    private boolean isDragging = false;

    public enum ServoPosition {
        SERVO_POSITION_OPEN,
        SERVO_POSITION_CLOSED
    }

    public LiftController(HardwareMap hardwareMap, Config config, Telemetry telemetry) {
        super(hardwareMap, config);
        this.telemetry = telemetry;
        lift = hardwareMap.dcMotor.get("lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        dragServo      = hardwareMap.servo.get("dragservo");
        leftLiftServo  = hardwareMap.servo.get("llservo");
        rightLiftServo = hardwareMap.servo.get("lrservo");
    }

    public void initLift() {
        lift.setTargetPosition(200);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(LIFT_SPEED);

        dragServo.setPosition(1);
        leftLiftServo.setPosition(1);
        rightLiftServo.setPosition(0);
    }

    public void stop() {
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void toggleLift() {
        if(lift.getCurrentPosition() > 0) {
            lift.setTargetPosition(0);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(LIFT_SPEED);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            lift.setTargetPosition((int) MAX_LIFT_HEIGHT_IN_TICKS);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(LIFT_SPEED);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void toggleDragServo() {
        if(dragServo.getPosition() > 0) {
            dragServo.setPosition(0);
        } else {
            dragServo.setPosition(1);
        }
    }

    public void setServoPosition(ServoPosition servoPosition) {
        if(servoPosition == ServoPosition.SERVO_POSITION_OPEN) {
            leftLiftServo.setPosition(0);
            rightLiftServo.setPosition(1);
        } else {
            leftLiftServo.setPosition(1);
            rightLiftServo.setPosition(0);
        }
    }

    @Override
    public void gamepadButtonEvent(FtcGamePad gamepad, int button, boolean pressed) {
        switch(button) {
            case FtcGamePad.GAMEPAD_DPAD_DOWN:
                if(pressed) {
                    if(lift.getCurrentPosition() >= 0) {
                        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lift.setPower(-LIFT_SPEED);
                    } else {
                        lift.setPower(0);
                    }
                } else{
                    lift.setPower(0);
                    lift.setTargetPosition(lift.getCurrentPosition());
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;
            case FtcGamePad.GAMEPAD_DPAD_UP:
                if(pressed) {
                    if(lift.getCurrentPosition() <= 455) {
                        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lift.setPower(LIFT_SPEED);
                    } else {
                        lift.setPower(0);
                    }
                } else {
                    lift.setPower(0);
                    lift.setTargetPosition(lift.getCurrentPosition());
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;
            case FtcGamePad.GAMEPAD_X:
                if(pressed) {
                    if (isClosed) {
                        leftLiftServo.setPosition(0.3);
                        rightLiftServo.setPosition(0.5);
                        isClosed = false;
                    } else {
                        leftLiftServo.setPosition(1);
                        rightLiftServo.setPosition(0);
                        isClosed = true;
                    }
                }
                break;
            case FtcGamePad.GAMEPAD_B:
                if(pressed) {
                    if(isDragging) {
                        dragServo.setPosition(0);
                    } else {
                        dragServo.setPosition(1);
                    }
                    isDragging = !isDragging;
                }
        }
    }
}
