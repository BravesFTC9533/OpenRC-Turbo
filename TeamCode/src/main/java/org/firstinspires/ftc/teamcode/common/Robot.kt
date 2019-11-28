package org.firstinspires.ftc.teamcode.common

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class Robot(hardwareMap: HardwareMap) {
    val WHEEL_DISTANCE_INCHES = 18.1

    val NEVE_COUNTS_PER_MOTOR_NEVE = 1120.0      // eg: Rev Side motor
    val DRIVE_GEAR_REDUCTION = 45.0 / 35.0             // This is < 1.0 if geared UP
    val WHEEL_DIAMETER_INCHES = 5.250           // For figuring circumference
    val COUNTS_PER_INCH = NEVE_COUNTS_PER_MOTOR_NEVE * DRIVE_GEAR_REDUCTION / (WHEEL_DIAMETER_INCHES * Math.PI)

    val fl: DcMotorEx
    val fr: DcMotorEx
    val bl: DcMotorEx
    val br: DcMotorEx

    private var isReverse = false

    init {
        fl = hardwareMap.get("fl") as DcMotorEx
        fr = hardwareMap.get("fr") as DcMotorEx
        bl = hardwareMap.get("bl") as DcMotorEx
        br = hardwareMap.get("br") as DcMotorEx

        fl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        fr.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        bl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        br.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        fr.direction = DcMotorSimple.Direction.REVERSE
        br.direction = DcMotorSimple.Direction.REVERSE
    }

    fun setMode(runMode: DcMotor.RunMode) {
        fl.mode = runMode
        fr.mode = runMode
        bl.mode = runMode
        br.mode = runMode
    }

    fun setReverse(value: Boolean) {
        if (value) {
            fl.direction = DcMotorSimple.Direction.REVERSE
            bl.direction = DcMotorSimple.Direction.REVERSE
            fr.direction = DcMotorSimple.Direction.FORWARD
            br.direction = DcMotorSimple.Direction.FORWARD
        } else {
            fl.direction = DcMotorSimple.Direction.FORWARD
            bl.direction = DcMotorSimple.Direction.FORWARD
            fr.direction = DcMotorSimple.Direction.REVERSE
            br.direction = DcMotorSimple.Direction.REVERSE
        }
        isReverse = value
    }

    fun isReverse() : Boolean {
        return isReverse
    }

    fun isBusy() : Boolean {
        return fl.isBusy || fr.isBusy || bl.isBusy || br.isBusy
    }

}