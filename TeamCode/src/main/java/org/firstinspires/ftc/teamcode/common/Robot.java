package org.firstinspires.ftc.teamcode.common;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot implements SensorEventListener {

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

    public float x, y, z;

    private Telemetry telemetry;

    public ColorSensor colorSensorLeft;
    public ColorSensor colorSensorRight;

    private SensorManager sensorManager;
    private Sensor accelerometer;

    private boolean isReverse = false;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
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

        sensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);

        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_NORMAL);

//        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
//        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorRight");
    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        x = sensorEvent.values[0];
        y = sensorEvent.values[1];
        z = sensorEvent.values[2];
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

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
