package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmsController {

    public final Servo leftArm;
    public final Servo rightArm;

    public enum ArmSide {
        LEFT,
        RIGHT
    }

    public ArmsController(HardwareMap hardwareMap) {
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");

        rightArm.setDirection(Servo.Direction.REVERSE);
    }

    public void init() {}

    public void toggleArm(ArmSide armSide) {
        switch (armSide) {
            case LEFT:
                if(leftArm.getPosition() > 0) {
                    leftArm.setPosition(0);
                } else {
                    leftArm.setPosition(0.8);
                }
                break;
            case RIGHT:
                if(rightArm.getPosition() > 0) {
                    rightArm.setPosition(0);
                } else {
                    rightArm.setPosition(1);
                }
                break;
        }
    }

    public void openArm(ArmSide armSide) {
        switch (armSide) {
            case LEFT:
                leftArm.setPosition(1);
                break;
            case RIGHT:
                rightArm.setPosition(1);
                break;
        }
    }

    public void closeArm(ArmSide armSide) {
        switch (armSide) {
            case LEFT:
                leftArm.setPosition(-1);
                break;
            case RIGHT:
                rightArm.setPosition(-1);
                break;
        }
    }

}
