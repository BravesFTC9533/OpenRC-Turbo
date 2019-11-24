package org.firstinspires.ftc.teamcode.adultbot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    public static final double WHEEL_DISTANCE_INCHES = 18.1;

    public static final double     COUNTS_PER_MOTOR_REV = 1120;      // eg: Rev Side motor
    public static final double     DRIVE_GEAR_REDUCTION    = 45.0 / 35.0;             // This is < 1.0 if geared UP
    public static final double     WHEEL_DIAMETER_INCHES   = 5.250;           // For figuring circumference
    public static final double     COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);



    public HardwareMap hardwareMap;

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public BNO055IMU imu;

    public Robot(HardwareMap hardwarMap) {
        this.hardwareMap = hardwarMap;
    }


    public void InitializeDrive(IDrive drive){
        DcMotorEx[] motors = new DcMotorEx[4];
        motors[0] = frontLeft;
        motors[1] = frontRight;
        motors[2] = backLeft;
        motors[3] = backRight;
        drive.AddMotors(motors);
    }

}
