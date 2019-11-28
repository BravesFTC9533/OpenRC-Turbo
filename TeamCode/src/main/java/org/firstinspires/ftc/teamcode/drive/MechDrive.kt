package org.firstinspires.ftc.teamcode.drive

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.common.FtcGamePad
import org.firstinspires.ftc.teamcode.common.Robot

class MechDrive(robot: Robot, opMode: LinearOpMode) : Drive(opMode) {

    private val MIN_SPEED = 0.2

    val robot = robot

    val fl = robot.fl
    val fr = robot.fr
    val bl = robot.bl
    val br = robot.br

    // Scale motor power based on the max for all wheels
    // 1, 1, 1, 3 will become .33, .33, .33, 1
    fun scalePower(value: Double, max: Double): Double {
        return if (max == 0.0) {
            0.0
        } else value / max
    }

    fun clipMotorPower(value: Double): Double {
        return Range.clip(value, -1.0, 1.0)
    }

    fun addTargetPosition(leftTicks: Int, rightTicks: Int) {
        fl.targetPosition = fl.currentPosition + leftTicks
        bl.targetPosition = bl.currentPosition + leftTicks
        fr.targetPosition = fr.currentPosition + rightTicks
        br.targetPosition = br.currentPosition + rightTicks
    }

    override fun drive(v: Double, h: Double, r: Double) {
        // add vectors
        var frontLeft = v - h + r
        var frontRight = v + h - r
        var backRight = v - h - r
        var backLeft = v + h + r

        // since adding vectors can go over 1, figure out max to scale other wheels
        val max = Math.max(
                Math.abs(backLeft),
                Math.max(
                        Math.abs(backRight),
                        Math.max(
                                Math.abs(frontLeft), Math.abs(frontRight)
                        )
                )
        )
        // only need to scale power if max > 1
        if (max > 1) {
            frontLeft = scalePower(frontLeft, max)
            frontRight = scalePower(frontRight, max)
            backLeft = scalePower(backLeft, max)
            backRight = scalePower(backRight, max)
        }

        fl.power = frontLeft
        fr.power = frontRight
        bl.power = backLeft
        br.power = backRight
    }

    override fun handleTeleop(driverGamepad: FtcGamePad) {
        var h: Double
        var v: Double
        var r: Double

        h = -driverGamepad.leftStickX
        v = -driverGamepad.leftStickY
        r = -driverGamepad.rightStickX

        if (Math.abs(h) < MIN_SPEED) {
            h = 0.0
        }
        if (Math.abs(v) < MIN_SPEED) {
            v = 0.0
        }
        if (Math.abs(r) < MIN_SPEED) {
            r = 0.0
        }

        if (robot.isReverse()) {
            h *= -1.0
            v *= -1.0
        }


        h = clipMotorPower(h)
        v = clipMotorPower(v)
        r = clipMotorPower(r)

        // add vectors
        var frontLeft = v - h + r
        var frontRight = v + h - r
        var backRight = v - h - r
        var backLeft = v + h + r

        // since adding vectors can go over 1, figure out max to scale other wheels
        val max = Math.max(
                Math.abs(backLeft),
                Math.max(
                        Math.abs(backRight),
                        Math.max(
                                Math.abs(frontLeft), Math.abs(frontRight)
                        )
                )
        )
        // only need to scale power if max > 1
        if (max > 1) {
            frontLeft = scalePower(frontLeft, max)
            frontRight = scalePower(frontRight, max)
            backLeft = scalePower(backLeft, max)
            backRight = scalePower(backRight, max)
        }

        fl.power = frontLeft
        fr.power = frontRight
        bl.power = backLeft
        br.power = backRight
    }

    override fun moveByInches(power: Double, inches: Float, timeoutSeconds: Double) {
        encoderDrive(power, (robot.COUNTS_PER_INCH * inches) as Int, (robot.COUNTS_PER_INCH * inches) as Int, timeoutSeconds)
    }

    override fun encoderDrive(power: Double, leftTicks: Int, rightTicks: Int, timeoutSeconds: Double) {
        addTargetPosition(leftTicks, rightTicks)
        robot.setMode(DcMotor.RunMode.RUN_TO_POSITION)

        drive(power, 0.0, 0.0)

        val runtime = ElapsedTime()
        runtime.reset()

        while (opMode.opModeIsActive() && robot.isBusy() && runtime.seconds() <= timeoutSeconds) {
        }

        stop()
    }

    override fun stop() {
        drive(0.0, 0.0, 0.0)
    }
}