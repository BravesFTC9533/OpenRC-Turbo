package org.firstinspires.ftc.teamcode.common

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.Drive
import org.firstinspires.ftc.teamcode.drive.MechDrive

open class BaseLinearOpMode : LinearOpMode() {

    lateinit var robot: Robot
    lateinit var drive: Drive

    override fun runOpMode() {
        robot = Robot(hardwareMap)
        drive = MechDrive(robot, this)

        waitForStart()
    }

}