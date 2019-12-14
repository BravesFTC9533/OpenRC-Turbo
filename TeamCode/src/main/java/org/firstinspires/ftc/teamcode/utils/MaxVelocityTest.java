package org.firstinspires.ftc.teamcode.utils;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Max Velocity", group="Util Opmode")
public class MaxVelocityTest extends LinearOpMode {

    DcMotorEx motor;
    double currentVelocity;
    double maxVelocity = 0.0;

    @Override

    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class, "fl");

        waitForStart();

        motor.setPower(1);

        while (opModeIsActive()) {

            currentVelocity = motor.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;

            }

            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Maximum Velocity", maxVelocity);
            telemetry.update();

        }

    }

}