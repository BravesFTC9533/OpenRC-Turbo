package org.firstinspires.ftc.teamcode.drive

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.common.FtcGamePad

abstract class Drive(opMode: LinearOpMode) {

    protected val opMode: LinearOpMode = opMode

    abstract fun drive(v: Double, h: Double, r: Double)
    abstract fun handleTeleop(gamePad: FtcGamePad)
    abstract fun moveByInches(power: Double, inches: Float, timeoutSeconds: Double)
    abstract fun encoderDrive(power: Double, leftTicks: Int, rightTicks: Int, timeoutSeconds: Double)
    abstract fun stop()

}