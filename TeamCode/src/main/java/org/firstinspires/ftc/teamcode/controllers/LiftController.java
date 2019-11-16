package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public Servo flipper;
    public Servo bServo;
    public Servo fServo;

    private boolean isClosed = true;
    private boolean isGrabbing = true;
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
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        flipper        = hardwareMap.servo.get("flipper");
        dragServo      = hardwareMap.servo.get("dragservo");
        fServo         = hardwareMap.servo.get("fservo");
        fServo.setDirection(Servo.Direction.REVERSE);
        bServo         = hardwareMap.servo.get("bservo");
    }

    public void initLift(LinearOpMode opMode) {
        dragServo.setPosition(1);
        flipper.setPosition(1);

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while(elapsedTime.seconds() < 3) {}

        bServo.setPosition(0.5);
        fServo.setPosition(0);
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

    public void setFlipperPosition(ServoPosition servoPosition) {
        switch (servoPosition) {
            case SERVO_POSITION_OPEN:
                flipper.setPosition(1);
                break;
            case SERVO_POSITION_CLOSED:
                flipper.setPosition(0);
                break;
        }
    }

    public void setGrabPosition(ServoPosition servoPosition) {
        switch (servoPosition) {
            case SERVO_POSITION_OPEN:
                bServo.setPosition(1);
                fServo.setPosition(1);
                break;
            case SERVO_POSITION_CLOSED:
                bServo.setPosition(0);
                fServo.setPosition(0);
        }
    }

    @Override
    public void gamepadButtonEvent(FtcGamePad gamepad, int button, boolean pressed) {
        telemetry.addData("Lift: ", lift.getCurrentPosition());
        switch(button) {
            case FtcGamePad.GAMEPAD_DPAD_DOWN:
                if(pressed) {
                    if(lift.getCurrentPosition() >= 10) {
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
                    if(lift.getCurrentPosition() <= 450) {
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
            case FtcGamePad.GAMEPAD_B:
                if(pressed) {
                    if(isDragging) {
                        dragServo.setPosition(0);
                    } else {
                        dragServo.setPosition(1);
                    }
                    isDragging = !isDragging;
                }
                break;
            case FtcGamePad.GAMEPAD_A:
                if(pressed) {
                    if(isClosed) {
                        flipper.setPosition(1);
                    } else {
                        flipper.setPosition(0.5);
                    }
                    isClosed = !isClosed;
                }
            case FtcGamePad.GAMEPAD_Y:
                if(pressed) {
                    if (isGrabbing) {
                        bServo.setPosition(0.5);
                        fServo.setPosition(0);
                    } else {
                        bServo.setPosition(1);
                        fServo.setPosition(1);
                    }
                    isGrabbing = !isGrabbing;
                }

        }
    }
}
