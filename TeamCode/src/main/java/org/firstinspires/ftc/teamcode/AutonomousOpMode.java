package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

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

    private Config config;
    private Sensors sensors;

    private Config.Position startingPosition;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Robot. Hands Clear!");
        telemetry.update();

        Initialize(hardwareMap);
        drive = new MecanumDrive(this, robot);

        config = new Config(hardwareMap.appContext);
        startingPosition = config.getPosition();

        liftController = new LiftController(hardwareMap, config, telemetry);

        sensors = new Sensors(hardwareMap);

        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();

        telemetry.addData("Status", "Initializing Vuforia");
        telemetry.update();

        try {
            initializeVuforia();
        } catch (Exception e) {
            telemetry.addData("Status", "Vuforia Could not be Initialized!");
            telemetry.update();
            return;
        }

        telemetry.addData("Status", "Vuforia Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        sensors.colorSensor.enableLed(true);

        while(opModeIsActive()) {

            telemetry.addData("Sensor ARGB: ", sensors.colorSensor.argb());
            telemetry.update();
        }


//        liftController.initLift(this);
//        liftController.putFlipperUp();
//
//        switch(startingPosition) {
//            case BLUE_BRICKS:
//                blueBricks();
//                break;
//            case BLUE_BUILDING:
//                blueBuilding();
//                break;
//            case RED_BRICKS:
//                redBricks();
//                break;
//            case RED_BUILDING:
//                redBuilding();
//                break;
//        }
    }

    boolean isTargetFound = false;

    private void redBricks() {
        drive.moveByInches(0.7, 4, 1.5);

        drive.turnDegrees(IDrive.TurnDirection.COUNTER_CLOCKWISE, 90, 0.6, 3);

        drive.moveByInches(0.7, 16, 3);

        drive.turnDegrees(IDrive.TurnDirection.COUNTER_CLOCKWISE, 90, 0.7, 2);

        drive.moveByInches(0.6, -20, 1.5);

        liftController.toggleDragServo();

        sleep(1000);

        if(config.getParkPosition() == Config.ParkPosition.BRIDGE) {
            drive.moveByInches(0.7, 10, 1.5);
        } else {
            drive.moveByInches(0.7, 20, 1.5);
        }

        drive.turnDegrees(IDrive.TurnDirection.COUNTER_CLOCKWISE, 90, 0.9, 2);

        drive.moveByInches(0.6, 60, 4.5);

        drive.turnDegrees(IDrive.TurnDirection.CLOCKWISE, 180, 0.7, 2.5);

        liftController.toggleDragServo();

        drive.moveByInches(0.6, 24, 2);

    }

    private void blueBricks() {
        drive.moveByInches(0.7, 4, 1.5);

        drive.turnDegrees(IDrive.TurnDirection.CLOCKWISE, 90, 0.6, 3);

        drive.moveByInches(0.7, 14, 3);

        drive.turnDegrees(IDrive.TurnDirection.CLOCKWISE, 90, 0.7, 2);

        drive.moveByInches(0.6, -20, 1.5);

        liftController.toggleDragServo();

        sleep(1000);

        if(config.getParkPosition() == Config.ParkPosition.BRIDGE) {
            drive.moveByInches(0.7, 10, 1.5);
        } else {
            drive.moveByInches(0.7, 20, 1.5);
        }

        drive.turnDegrees(IDrive.TurnDirection.CLOCKWISE, 90, 0.9, 2);

        drive.moveByInches(0.6, 60, 4.5);

        drive.turnDegrees(IDrive.TurnDirection.COUNTER_CLOCKWISE, 180, 0.7, 2.5);

        liftController.toggleDragServo();

        drive.moveByInches(0.6, 24, 2);

    }

    private void redBuilding() {
        drive.moveByInches(0.7, -15, 1.3);

        drive.turnDegrees(IDrive.TurnDirection.CLOCKWISE, 90, 0.6, 1.5);

        drive.moveByInches(0.7, -18.5, 1.3);

        drive.turnDegrees(IDrive.TurnDirection.COUNTER_CLOCKWISE, 90, 0.6, 1.5);

        drive.moveByInches(0.7, -6, 2);

        liftController.toggleDragServo();

        sleep(400);

        drive.moveByInches(0.7, 30, 2);

        liftController.toggleDragServo();

        sleep(400);

        if(config.getParkPosition() == Config.ParkPosition.BRIDGE) {
            drive.moveByInches(0.7, 20, 1.5);
        }

        runtime.reset();
        while(runtime.seconds() < 4) {
            drive.drive(0, 0.5, 0);
        }
        drive.stop();
    }

    private void blueBuilding() {
        drive.moveByInches(0.7, -15, 1.3);

        drive.turnDegrees(IDrive.TurnDirection.COUNTER_CLOCKWISE, 90, 0.6, 1.5);

        drive.moveByInches(0.7, -18.5, 1.3);

        drive.turnDegrees(IDrive.TurnDirection.CLOCKWISE, 90, 0.6, 1.5);

        drive.moveByInches(0.7, -6, 2);

        liftController.toggleDragServo();

        sleep(400);

        drive.moveByInches(0.7, 30, 2);

        liftController.toggleDragServo();

        sleep(400);

        if(config.getParkPosition() == Config.ParkPosition.BRIDGE) {
            drive.moveByInches(0.7, 20, 1.5);
        }

        runtime.reset();
        while(runtime.seconds() < 4.25) {
            drive.drive(0, -0.5, 0);
        }
        drive.stop();
    }
}
