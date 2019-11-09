package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class BaseController {

    protected HardwareMap hardwareMap;
    protected Config config;

    public BaseController(HardwareMap hardwareMap, Config config) {
        this.hardwareMap = hardwareMap;
        this.config = config;
    }

    public abstract void gamepadButtonEvent(FtcGamePad gamepad, int button, boolean pressed);
}
