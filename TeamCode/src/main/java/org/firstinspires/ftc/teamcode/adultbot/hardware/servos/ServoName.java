package org.firstinspires.ftc.teamcode.adultbot.hardware.servos;

import org.firstinspires.ftc.teamcode.adultbot.hardware.config.RobotCfg;

public interface ServoName {
    /**
     * @return the name as defined in the hardwareMap
     * @see com.qualcomm.robotcore.hardware.HardwareMap
     */
    String getHardwareName();

    /**
     * @return the names of the presets for this servo
     */
    Enum[] getPresets();

    /**
     * @return the name for telemetry purposes
     */
    String name();

    /**
     * @return the subclass of RobotCfg that this servo is associated with
     */
    Class<? extends RobotCfg> getRobotCfg();
}

