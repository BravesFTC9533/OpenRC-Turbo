package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.BaseLinearOpMode
import org.firstinspires.ftc.teamcode.common.FtcGamePad

@TeleOp(name="Teleop", group="Linear Opmode")
class Teleop : BaseLinearOpMode(), FtcGamePad.ButtonHandler {

    lateinit var driverGamePad: FtcGamePad
    lateinit var operatorGamePad: FtcGamePad

    val runtime = ElapsedTime()

    override fun runOpMode() {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        driverGamePad = FtcGamePad("Driver Gampad", gamepad1, this)
        operatorGamePad = FtcGamePad("Operator Gampad", gamepad2, this)

        waitForStart()
        runtime.reset()

        while(opModeIsActive()) {
            drive.handleTeleop(driverGamePad)
            driverGamePad.update()
            operatorGamePad.update()
            telemetry.update()
        }
    }

    override fun gamepadButtonEvent(gamepad: FtcGamePad?, button: Int, pressed: Boolean) {
        if(gamepad == driverGamePad) {
            handleDriverGampad(button, pressed)
        } else if(gamepad == operatorGamePad) {
            handleOperatorGamepad(button, pressed)
        }
    }

    fun handleDriverGampad(button: Int, pressed: Boolean) {

    }

    fun handleOperatorGamepad(button: Int, pressed: Boolean) {

    }
}