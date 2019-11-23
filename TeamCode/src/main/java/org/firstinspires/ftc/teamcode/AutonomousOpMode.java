package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.drive.IDrive;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.sensors.Sensors;

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

        sensors.colorSensor.enableLed(false);

//        while(opModeIsActive()) {
//            telemetry.addData("Red", sensors.colorSensor.red());
//            telemetry.addData("Blue", sensors.colorSensor.blue());
//            telemetry.addData("Green", sensors.colorSensor.green());
//            telemetry.update();
//        }

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
        drive.moveByInches(0.8, 15, 1.5);

        drive.turnDegrees(IDrive.TurnDirection.CLOCKWISE, 90, 0.8, 1.5);

        drive.moveByInches(0.8, 23, 1.25);

        ((MecanumDrive) drive).strafe(MecanumDrive.StrafeDirection.RIGHT, 0.5, 0.75f);

        drive.drive(-0.3, -0.1, 0);

        while(opModeIsActive()) {
            if(sensors.isSkyStone()) {
                drive.stop();
                break;
            }
        }

        telemetry.log().add("Grabbed the brick.");
        telemetry.update();

        ((MecanumDrive) drive).strafe(MecanumDrive.StrafeDirection.LEFT, 0.5, 1.5f);

        drive.moveByInches(0.7, -35, 1.5);
    }

    private void blueBricks() {


    }

    private void redBuilding() {

    }

    private void blueBuilding() {

    }
}
