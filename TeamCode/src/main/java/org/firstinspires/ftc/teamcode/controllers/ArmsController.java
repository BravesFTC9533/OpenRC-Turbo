package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.FtcGamePad;

public class ArmsController {

    public final Servo leftArm;
    public final Servo rightArm;
    public final Servo foundationArm;

    public static final double MAX_LEFT_ARM_POSITION = 0.8;
    public static final double MAX_RIGHT_ARM_POSITION = 1;
    public static final double MIN_LEFT_ARM_POSITION = -1;
    public static final double MIN_RIGHT_ARM_POSITION = -1;


    public enum ArmSide {
        LEFT,
        RIGHT
    }

    public ArmsController(HardwareMap hardwareMap) {
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");

        leftArm.setDirection(Servo.Direction.REVERSE);
        rightArm.setDirection(Servo.Direction.FORWARD);

        foundationArm = hardwareMap.get(Servo.class, "foundation");
        foundationArm.setDirection(Servo.Direction.REVERSE);
    }

    public void init() {
        rightArm.setPosition(MAX_RIGHT_ARM_POSITION);
        leftArm.setPosition(MAX_LEFT_ARM_POSITION);
        releaseFoundation();
    }

    public void grabFoundation() {
        foundationArm.setPosition(1);
    }

    public void releaseFoundation() {
        foundationArm.setPosition(-1);
    }

    public void toggleArm(ArmSide armSide) {
        switch (armSide) {
            case LEFT:
                if(leftArm.getPosition() > MIN_LEFT_ARM_POSITION) {
                    leftArm.setPosition(MIN_LEFT_ARM_POSITION);
                } else {
                    leftArm.setPosition(MAX_LEFT_ARM_POSITION);
                }
                break;
            case RIGHT:
                if(rightArm.getPosition() > MIN_RIGHT_ARM_POSITION) {
                    rightArm.setPosition(MIN_RIGHT_ARM_POSITION);
                } else {
                    rightArm.setPosition(MAX_RIGHT_ARM_POSITION);
                }
                break;
        }
    }

    public void openArm(ArmSide armSide) {
        switch (armSide) {
            case LEFT:
                leftArm.setPosition(MAX_LEFT_ARM_POSITION);
                break;
            case RIGHT:
                rightArm.setPosition(MAX_RIGHT_ARM_POSITION);
                break;
        }
    }

    public void closeArm(ArmSide armSide) {
        switch (armSide) {
            case LEFT:
                leftArm.setPosition(MIN_LEFT_ARM_POSITION);
                break;
            case RIGHT:
                rightArm.setPosition(MIN_RIGHT_ARM_POSITION);
                break;
        }
    }

    public void handle(int button, boolean pressed) {
        switch (button) {
            case FtcGamePad.GAMEPAD_X:
                if(pressed) {
                    if (foundationArm.getPosition() > 0) {
                        foundationArm.setPosition(-1);
                    } else {
                        foundationArm.setPosition(1);
                    }
                }
                break;
        }
    }

}
