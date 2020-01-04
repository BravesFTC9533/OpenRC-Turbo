package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftController {

    private final DcMotor lift;

    public LiftController(HardwareMap hardwareMap) {
        lift = hardwareMap.dcMotor.get("lift");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setLiftPower(double power) {
        lift.setPower(power);
    }

    public void handle(int button, boolean pressed) {
        switch (button) {
            case FtcGamePad.GAMEPAD_DPAD_UP:
                if(pressed)
                    setLiftPower(1);
                else
                    holdPosition();
                break;
            case FtcGamePad.GAMEPAD_DPAD_DOWN:
                if(pressed)
                    setLiftPower(-1);
                else
                    holdPosition();
                break;
        }
    }

    public void holdPosition() {
        lift.setTargetPosition(lift.getTargetPosition());
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setLiftPower(1);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
