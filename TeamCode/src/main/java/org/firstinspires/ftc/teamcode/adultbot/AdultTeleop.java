package org.firstinspires.ftc.teamcode.adultbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.common.FtcGamePad;


@TeleOp(name="Adult Bot!!", group="Java")
public class AdultTeleop extends LinearOpMode implements FtcGamePad.ButtonHandler {



    // Add access to the robot class
    protected Robot robot;

    private FtcGamePad driverGamePad;


    MecDrive drive;
    @Override public void runOpMode() {

        robot = new Robot(hardwareMap);

        drive = new MecDrive();

        robot.InitializeDrive(drive);

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        driverGamePad = new FtcGamePad("Driver", gamepad1, this);

        /* we keep track of how long it's been since the OpMode was started, just
         * to have some interesting data to show */
        ElapsedTime opmodeRunTime = new ElapsedTime();

        // We show the log in oldest-to-newest order, as that's better for poetry
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
        // We can control the number of lines shown in the log
        telemetry.log().setCapacity(6);


        /**
         * Wait until we've been given the ok to go. For something to do, we emit the
         * elapsed time as we sit here and wait. If we didn't want to do anything while
         * we waited, we would just call {@link #waitForStart()}.
         */
        while (!isStarted()) {
            telemetry.addData("time", "%.1f seconds", opmodeRunTime.seconds());
            telemetry.update();
            idle();
        }

        // Ok, we've been given the ok to go

        /**
         * As an illustration, the first line on our telemetry display will display the battery voltage.
         * The idea here is that it's expensive to compute the voltage (at least for purposes of illustration)
         * so you don't want to do it unless the data is <em>actually</em> going to make it to the
         * driver station (recall that telemetry transmission is throttled to reduce bandwidth use.
         * Note that getBatteryVoltage() below returns 'Infinity' if there's no voltage sensor attached.
         *
         * @see Telemetry#getMsTransmissionInterval()
         */
        telemetry.addData("voltage", "%.1f volts", new Func<Double>() {
            @Override public Double value() {
                return getBatteryVoltage();
            }
        });

        // Reset to keep some timing stats for the post-'start' part of the opmode
        opmodeRunTime.reset();
        int loopCount = 1;

        // Go go gadget robot!
        while (opModeIsActive()) {


            driverGamePad.update();
            drive.handle(driverGamePad);

            // As an illustration, show some loop timing information
            telemetry.addData("loop count", loopCount);
            telemetry.addData("ms/loop", "%.3f ms", opmodeRunTime.milliseconds() / loopCount);

            telemetry.addData("mode", drive.getDriveMode().name());

//            telemetry.addData("reverse", new Func<String>() {
//                        @Override public String value() {
//                            return Boolean.toString( drive.getIsReverse());
//                        }
//                    });

            /**
             * Transmit the telemetry to the driver station, subject to throttling.
             * @see Telemetry#getMsTransmissionInterval()
             */
            telemetry.update();


        }
    }


    // Computes the current battery voltage
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    @Override
    public void gamepadButtonEvent(FtcGamePad gamepad, int button, boolean pressed) {
        switch(button) {


            case FtcGamePad.GAMEPAD_B:

                if(pressed) {
                   drive.setIsReverse(!drive.isReverse);
                }
                break;
            case FtcGamePad.GAMEPAD_A:
                if(pressed) {
                    drive.changeDriveMode();
                }
                break;
            case FtcGamePad.GAMEPAD_Y:
                if(pressed) {

                }
                break;
        }
    }
}
