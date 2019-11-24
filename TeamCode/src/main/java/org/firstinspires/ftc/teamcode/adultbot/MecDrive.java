package org.firstinspires.ftc.teamcode.adultbot;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.Arrays;

public class MecDrive implements IDrive {

    ArrayList<DcMotorEx> motors;


    @Override
    public void AddMotors(DcMotorEx[] motors) {
        this.motors = new ArrayList<>(Arrays.asList(motors));
    }
}
