package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

public class Robot {

    public static final double WHEEL_DISTANCE_INCHES = 18.1;

    public static final double     NEVE_COUNTS_PER_MOTOR_NEVE = 1120;      // eg: Rev Side motor
    public static final double            DRIVE_GEAR_REDUCTION    = 45.0 / 35.0;             // This is < 1.0 if geared UP
    public static final double            WHEEL_DIAMETER_INCHES   = 5.250;           // For figuring circumference
    public static final double     COUNTS_PER_INCH = (NEVE_COUNTS_PER_MOTOR_NEVE * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    public final DcMotorEx fl;
    public final DcMotorEx fr;
    public final DcMotorEx bl;
    public final DcMotorEx br;

    public final ColorSensor colorSensorLeft;
    public final ColorSensor colorSensorRight;

    private boolean isReverse = false;

    public Robot(HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);

        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorRight");
    }

    public void setMode(DcMotor.RunMode runMode) {
        fl.setMode(runMode);
        fr.setMode(runMode);
        bl.setMode(runMode);
        br.setMode(runMode);
    }

    public void setReverse(boolean value) {
        if(value) {
            fl.setDirection(DcMotorEx.Direction.REVERSE);
            bl.setDirection(DcMotorEx.Direction.REVERSE);
            fr.setDirection(DcMotorEx.Direction.FORWARD);
            br.setDirection(DcMotorEx.Direction.FORWARD);
        } else {
            fl.setDirection(DcMotorEx.Direction.FORWARD);
            bl.setDirection(DcMotorEx.Direction.FORWARD);
            fr.setDirection(DcMotorEx.Direction.REVERSE);
            br.setDirection(DcMotorEx.Direction.REVERSE);
        }
        isReverse = value;
    }

    public boolean isReverse() {
        return isReverse;
    }

    public boolean isBusy() {
        return fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy();
    }

}
