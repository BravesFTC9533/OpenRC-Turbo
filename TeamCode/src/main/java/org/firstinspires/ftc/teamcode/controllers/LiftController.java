package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    public static final int MAX_LIFT_HEIGHT_IN_TICKS = 400*2;
    public static final int MIN_LIFT_HEIGHT_IN_TICKS = 0;


    public static final int LIFT_STAGE_GRAB = 50;
    public static final int LIFT_STAGE_1 = 133*2;
    public static final int LIFT_STAGE_2 = 266*2;
    public static final int LIFT_STAGE_3 = 400*2;

    public static double LIFT_SPEED_UP = 600;
    public static double LIFT_SPEED_DOWN = 288;

    public static final double OPEN_GRABBER_DELAY = 2.0;

    public DcMotorEx lift;
    private Telemetry telemetry;



    private static final double FLIPPER_UP_POS = 0.5;
    private static final double FLIPPER_DOWN_POS = 1.0;

    private static final double DRAG_UP_POS = 0.0;
    private static final double DRAG_DOWN_POS = 1.0;

    public Servo dragServo;
    public Servo flipper;
    public Servo bServo;
    public Servo fServo;

    private boolean isFlipperDown = true;
    private boolean isGrabbing = true;
    private boolean isDragging = false;
    private boolean isMoving = false;
    private boolean initialServoHandled = false;
    private ElapsedTime runTime;
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
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

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

        runTime = new ElapsedTime();
    }

    public void initLift(LinearOpMode opMode) {

        dragServo.setPosition(1);

        //put flipper in down position
        flipper.setPosition(FLIPPER_DOWN_POS);


        runTime.reset();

    }

    public void stop() {
        lift.setVelocity(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void handle() {

        if(runTime.seconds() >= OPEN_GRABBER_DELAY && initialServoHandled == false) {
            initialServoHandled =true;
            openGrabbers();
        }

        if(lift.isBusy() == false && isMoving == true) {

            lift.setVelocity(0);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            isMoving = false;
        }
    }



    public void putDragArmDown() {
        dragServo.setPosition(DRAG_DOWN_POS);
    }
    public void putDragArmUp() {
        dragServo.setPosition(DRAG_UP_POS);
    }

    public void putFlipperDown() {
        flipper.setPosition(FLIPPER_DOWN_POS);
    }
    public void putFlipperUp() {
        flipper.setPosition(FLIPPER_UP_POS);
    }

    public void toggleDragServo() {
        if(dragServo.getPosition() > 0) {
            putDragArmUp();
        } else {
            putDragArmDown();
        }
    }

    private void openGrabbers() {
        bServo.setPosition(0.5);
        fServo.setPosition(0);
    }
    private void closeGrabbers() {
        bServo.setPosition(1);
        fServo.setPosition(1);
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
                        lift.setVelocity(LIFT_SPEED_DOWN);
                    }
                    else {
                        lift.setVelocity(LIFT_SPEED_UP);
                    }
                }
                break;
            case FtcGamePad.GAMEPAD_DPAD_UP:
                if(pressed) {
                    isMoving = true;
                    lift.setTargetPosition(LIFT_STAGE_2);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if(lift.getCurrentPosition() > LIFT_STAGE_1) {
                        lift.setVelocity(LIFT_SPEED_DOWN);
                    }
                    else {
                        lift.setVelocity(LIFT_SPEED_UP);
                    }
                }
                break;
            case FtcGamePad.GAMEPAD_DPAD_RIGHT:
                if(pressed) {
                    isMoving = true;
                    lift.setTargetPosition(LIFT_STAGE_1);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if(lift.getCurrentPosition() > LIFT_STAGE_3) {
                        lift.setVelocity(LIFT_SPEED_DOWN);
                    }
                    else {
                        lift.setVelocity(LIFT_SPEED_UP);
                    }
                }
                break;
            case FtcGamePad.GAMEPAD_DPAD_DOWN:
                if(pressed) {
                    lift.setTargetPosition(MIN_LIFT_HEIGHT_IN_TICKS);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //lift.setVelocity(LIFT_SPEED_DOWN);

                    lift.setVelocity(LIFT_SPEED_DOWN);
                }

                break;
                case FtcGamePad.GAMEPAD_LBUMPER:
                    if(pressed){
                        lift.setTargetPosition(LIFT_STAGE_GRAB);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        if(lift.getCurrentPosition() > LIFT_STAGE_GRAB) {
                            lift.setVelocity(LIFT_SPEED_DOWN);
                        }
                        else {
                            lift.setVelocity(LIFT_SPEED_UP);
                        }
                    }

                break;
            case FtcGamePad.GAMEPAD_B:

                if(pressed) {
                    if(isDragging) {
                        putDragArmUp();
                    } else {
                        //dragServo.setPosition(DRAG_DOWN_POS);
                        putDragArmDown();
                    }
                    isDragging = !isDragging;
                }
                break;
            case FtcGamePad.GAMEPAD_A:
                if(pressed) {
                    if(isFlipperDown) {
                        putFlipperUp();
                    } else {
                        putFlipperDown();
                    }
                    isFlipperDown = !isFlipperDown;
                }
                break;
            case FtcGamePad.GAMEPAD_Y:
                if(pressed) {
                    if (isGrabbing) {
                        openGrabbers();
                    } else {
                        closeGrabbers();
                    }
                    isGrabbing = !isGrabbing;
                }
                break;
        }
    }
}
