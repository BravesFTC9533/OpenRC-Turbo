package org.firstinspires.ftc.teamcode.common

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

class HardwareManager {

    private val hardwareMap: HardwareMap

    constructor(hardwareMap: HardwareMap) {
        this.hardwareMap = hardwareMap
    }

    fun getMotor(deviceName: String) : DcMotorEx {
        //return hardwareMap.get(DcMotorEx::class.java, deviceName)
        return get(DcMotorEx::class.java, deviceName)
    }

    fun <T> get(classOrInterface: Class<out T>, deviceName: String) : T {
        return hardwareMap.get<T>(classOrInterface, deviceName)
    }




}