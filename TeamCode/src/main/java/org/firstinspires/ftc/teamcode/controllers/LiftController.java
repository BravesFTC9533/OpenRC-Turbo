package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.BaseController;
import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.common.FtcGamePad;

public class LiftController extends BaseController {

    public static final int MAX_LIFT_HEIGHT_IN_TICKS = 400;
    public static final int MIN_LIFT_HEIGHT_IN_TICKS = 10;


    public static final int LIFT_STAGE_1 = 133;
    public static final int LIFT_STAGE_2 = 266;
    public static final int LIFT_STAGE_3 = 400;

    public static double LIFT_SPEED_UP = 1.00;
    public static double LIFT_SPEED_DOWN = 0.05;

    public DcMotorEx lift;
    private Telemetry telemetry;

    public Servo dragServo;
    public Servo flipper;
    public Servo bServo;
    public Servo fServo;

    private boolean isClosed = true;
    private boolean isGrabbing = true;
    private boolean isDragging = false;
    private boolean isMoving = false;

    public enum ServoPosition {
        SERVO_POSITION_OPEN,
        SERVO_POSITION_CLOSED
    }

    public LiftController(HardwareMap hardwareMap, Config config, Telemetry telemetry) {
        super(hardwareMap, config);
        this.telemetry = telemetry;
        lift = hardwareMap.get(DcMotorEx.class, "lift");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        //F = 32767 / 600 = 54.6
        //P = 0.1 * F = 5.46
        //I = 0.1 * P = 0.55
        //D = 0

        lift.setVelocityPIDFCoefficients(5.46, 0.55, 0, 54.6);
        lift.setPositionPIDFCoefficients(5.0);


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

        //while(elapsedTime.seconds() < 3) {}

        bServo.setPosition(0.5);
        fServo.setPosition(0);
    }

    public void stop() {
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void handle() {
        if(lift.isBusy() == false && isMoving == true) {

            lift.setPower(0);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            isMoving = false;
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

            case FtcGamePad.GAMEPAD_DPAD_LEFT:
                if(pressed) {
                    isMoving = true;
                    lift.setTargetPosition(LIFT_STAGE_1);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if(lift.getCurrentPosition() > LIFT_STAGE_1) {
                        lift.setPower(LIFT_SPEED_DOWN);
                    }
                    else {
                        lift.setPower(LIFT_SPEED_UP);
                    }
                }
                break;
            case FtcGamePad.GAMEPAD_DPAD_UP:
                if(pressed) {
                    isMoving = true;
                    lift.setTargetPosition(LIFT_STAGE_2);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if(lift.getCurrentPosition() > LIFT_STAGE_2) {
                        lift.setPower(LIFT_SPEED_DOWN);
                    }
                    else {
                        lift.setPower(LIFT_SPEED_UP);
                    }
                }
                break;
            case FtcGamePad.GAMEPAD_DPAD_RIGHT:
                if(pressed) {
                    isMoving = true;
                    lift.setTargetPosition(LIFT_STAGE_3);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if(lift.getCurrentPosition() > LIFT_STAGE_3) {
                        lift.setPower(LIFT_SPEED_DOWN);
                    }
                    else {
                        lift.setPower(LIFT_SPEED_UP);
                    }
                }
                break;
            case FtcGamePad.GAMEPAD_DPAD_DOWN:
                if(pressed) {
                    lift.setTargetPosition(MIN_LIFT_HEIGHT_IN_TICKS);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //lift.setPower(LIFT_SPEED_DOWN);

                    lift.setVelocity(288);
                }

                break;
//            case FtcGamePad.GAMEPAD_DPAD_DOWN:
//                if(pressed) {
//                    if(lift.getCurrentPosition() >= MIN_LIFT_HEIGHT_IN_TICKS) {
//                        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        lift.setPower(-LIFT_SPEED_DOWN);
//                    } else {
//                        lift.setPower(0);
//                    }
//                } else{
//                    lift.setPower(0);
//                    lift.setTargetPosition(lift.getCurrentPosition());
//                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
//                break;
//            case FtcGamePad.GAMEPAD_DPAD_UP:
//                if(pressed) {
//                    if(lift.getCurrentPosition() <= MAX_LIFT_HEIGHT_IN_TICKS) {
//                        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        lift.setPower(LIFT_SPEED_UP);
//                    } else {
//                        lift.setPower(0);
//                    }
//                } else {
//                    lift.setPower(0);
//                    lift.setTargetPosition(lift.getCurrentPosition());
//                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
//                break;
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
