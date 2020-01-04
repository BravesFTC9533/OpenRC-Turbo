package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmsController {

    public final Servo frontArm;
    public final Servo backArm;

    public enum ArmSide {
        FRONT,
        BACK
    }

    public ArmsController(HardwareMap hardwareMap) {
        frontArm = hardwareMap.get(Servo.class, "frontArm");
        backArm = hardwareMap.get(Servo.class, "backArm");

        frontArm.setPosition(0.2);
    }

    public void init() {
        frontArm.setPosition(1);
        backArm.setPosition(1);
    }

    public void toggleArm(ArmSide armSide) {
        switch (armSide) {
            case FRONT:
                if(frontArm.getPosition() > 0.2) {
                    frontArm.setPosition(0.2);
                } else {
                    frontArm.setPosition(1);
                }
                break;
            case BACK:
                if(backArm.getPosition() > 0) {
                    backArm.setPosition(0);
                } else {
                    backArm.setPosition(1);
                }
                break;
        }
    }

    public void openArm(ArmSide armSide) {
        switch (armSide) {
            case FRONT:
                frontArm.setPosition(1);
                break;
            case BACK:
                backArm.setPosition(1);
                break;
        }
    }

    public void closeArm(ArmSide armSide) {
        switch (armSide) {
            case FRONT:
                frontArm.setPosition(0);
                break;
            case BACK:
                backArm.setPosition(0);
                break;
        }
    }

}
