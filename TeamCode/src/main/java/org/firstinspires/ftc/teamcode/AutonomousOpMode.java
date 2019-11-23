package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.drive.IDrive;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Autonomous(name = "Java: Autonomous", group = "Concept")
public class AutonomousOpMode extends BaseLinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private Sensors sensors;

    private Config.ParkPosition parkPosition;

    private Config.Position startingPosition;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Robot. Hands Clear!");
        telemetry.update();

        Initialize(hardwareMap);
        drive = new MecanumDrive(this, robot);

        liftController = new LiftController(hardwareMap, config, telemetry);

        sensors = new Sensors(hardwareMap);

        telemetry.addData("Status", "Setting the configuration variables.");
        telemetry.update();

        parkPosition = config.getParkPosition();
        startingPosition = config.getPosition();

        telemetry.addData("Park Position", parkPosition);
        telemetry.addData("Starting Position", startingPosition);
        telemetry.update();

        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        liftController.initLift(this);
        liftController.putFlipperUp();

        switch(startingPosition) {
            case BLUE_BRICKS:
                blueBricks();
                break;
            case BLUE_BUILDING:
                blueBuilding();
                break;
            case RED_BRICKS:
                redBricks();
                break;
            case RED_BUILDING:
                redBuilding();
                break;
        }
    }

    private void redBricks() {
        // Move Off Wall
        drive.moveByInches(0.7, 10, 1.5);

        // Turn towards SkyBridge
        drive.turnDegrees(IDrive.TurnDirection.CLOCKWISE, 90, 0.7, 1.5);

        // Move backwards into the wall
        drive.moveByInches(0.8, -30, 2);

        // Straif close to the bricks.
        ((MecanumDrive) drive).straif(MecanumDrive.StraifDirection.LEFT, 0.5, 1);

        // Save the position where the robot started
        int startingPosition = robot.frontLeft.getCurrentPosition();

        // Start moving forward
        drive.drive(0.5, 0, 0);

        // Turn on color sensor LED
        sensors.colorSensor.enableLed(true);

        // Scan for SkyStone
        while(opModeIsActive()) {
            if(sensors.isSkyStone()) {
                drive.stop();
                break;
            }
        }

        // Turn off the color sensor LED
        sensors.colorSensor.enableLed(false);

        // Figure out the distance the robot moved
        int movedAmount = startingPosition - robot.frontLeft.getCurrentPosition();
        double movedAmountInches = movedAmount * Robot.COUNTS_PER_INCH;

        // Grab the SkyStone
        liftController.toggleDragServo();

        // Give the robot time to move the servo arm
        sleep(500);

        float straifTime = 0;
        if(parkPosition == Config.ParkPosition.WALL) {
            straifTime = 2;
            ((MecanumDrive) drive).straif(MecanumDrive.StraifDirection.RIGHT, 0.5, straifTime);
        } else {
            straifTime = 1;
            ((MecanumDrive) drive).straif(MecanumDrive.StraifDirection.RIGHT, 0.5, straifTime);
        }

        // Move across the tape
        drive.moveByInches(0.7, 50 - movedAmountInches, 2);

        // Drop off the SkyStone
        liftController.toggleDragServo();

        // Save the position where the robot started
        startingPosition = robot.frontLeft.getCurrentPosition();

        // Drive Backwards over the tape
        drive.drive(0, -0.5, 0);

        // Turn on the color sensor LED
        sensors.colorSensor.enableLed(true);

        while(opModeIsActive()) {
            if(sensors.isSkyStone()) {
                drive.stop();
                break;
            }
        }

        // Turn off the color sensor LED
        sensors.colorSensor.enableLed(false);

        // Straif next to the brick
        ((MecanumDrive) drive).straif(MecanumDrive.StraifDirection.LEFT, 0.5, straifTime);

        // Grab the last SkyStone
        liftController.toggleDragServo();

        // Give robot time
        sleep(500);

        // Figure movement
        movedAmount = startingPosition - robot.frontLeft.getCurrentPosition();
        movedAmountInches = movedAmount * Robot.COUNTS_PER_INCH;

        // Straif over
        if(parkPosition == Config.ParkPosition.WALL) {
            straifTime = 2;
            ((MecanumDrive) drive).straif(MecanumDrive.StraifDirection.RIGHT, 0.5, straifTime);
        } else {
            straifTime = 1;
            ((MecanumDrive) drive).straif(MecanumDrive.StraifDirection.RIGHT, 0.5, straifTime);
        }

        // Move Across the tape
        drive.moveByInches(0.8, 50 - movedAmountInches, 2);

        // Drop off the final SkyStone
        liftController.toggleDragServo();

        // Park on tape
        drive.moveByInches(1, -10, 1);
    }

    private void blueBricks() {


    }

    private void redBuilding() {

    }

    private void blueBuilding() {

    }
}
