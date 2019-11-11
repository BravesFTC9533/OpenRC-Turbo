package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.common.FtcGamePad;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Autonomous(name = "Java: Autonomous", group = "Concept")
public class AutonomousOpMode extends BaseLinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private Config config;

    private Config.Position startingPosition;
    private BrickPosition brickPosition;

    public enum BrickPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Robot. Hands Clear!");
        telemetry.update();

        Initialize(hardwareMap);
        drive = new MecanumDrive(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight);

        config = new Config(hardwareMap.appContext);
        startingPosition = config.getPosition();

        liftController = new LiftController(hardwareMap, config, telemetry);

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

        liftController.initLift();

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

    boolean isTargetFound = false;

    private void redBricks() {
        moveByInches(0.7, 4, 1.5);

        turnDegrees(TurnDirection.COUNTER_CLOCKWISE, 90, 0.6, 3);

        moveByInches(0.7, 16, 3);

        turnDegrees(TurnDirection.CLOCKWISE, 90, 0.7, 3);

        moveByInches(0.7, 5, 1.5);

        turnDegrees(TurnDirection.CLOCKWISE, 180, 0.7, 3);

        moveByInches(0.6, -15, 1.5);

        liftController.toggleDragServo();

        sleep(1000);

        moveByInches(0.7, 15, 1.5);

        turnDegrees(TurnDirection.COUNTER_CLOCKWISE, 100, 0.9, 2);

        moveByInches(0.6, 70, 5);

        turnDegrees(TurnDirection.CLOCKWISE, 90, 0.7, 2);

        liftController.toggleDragServo();

        turnDegrees(TurnDirection.CLOCKWISE, 90, 0.7, 3);

        moveByInches(0.6, 26, 3);

    }

    private void blueBricks() {
        moveByInches(0.7, 4, 1.5);

        turnDegrees(TurnDirection.CLOCKWISE, 90, 0.6, 3);

        moveByInches(0.7, 14, 3);

        turnDegrees(TurnDirection.COUNTER_CLOCKWISE, 90, 0.7, 3);

        moveByInches(0.7, 5, 1.5);

        turnDegrees(TurnDirection.CLOCKWISE, 180, 0.7, 3);

        moveByInches(0.6, -15, 1.5);

        liftController.toggleDragServo();

        sleep(1000);

        moveByInches(0.7, 14, 1.5);

        turnDegrees(TurnDirection.CLOCKWISE, 100, 0.9, 2);

        moveByInches(0.6, 70, 5);

        turnDegrees(TurnDirection.COUNTER_CLOCKWISE, 90, 0.7, 2);

        liftController.toggleDragServo();

        turnDegrees(TurnDirection.COUNTER_CLOCKWISE, 90, 0.7, 3);

        moveByInches(0.6, 26, 3);

    }

    private void redBuilding() {
        moveByInches(0.6, 10, 2);

        turnDegrees(TurnDirection.CLOCKWISE, 90, 0.6, 1.5);

        moveByInches(0.6, 17, 2);

        turnDegrees(TurnDirection.CLOCKWISE, 90, 0.6, 1.5);

        moveByInches(0.6, -11, 1.5);

        liftController.toggleDragServo();

        sleep(500);

        moveByInches(0.6, 26, 1.5);

        liftController.toggleDragServo();

        sleep(500);

        runtime.reset();
        while(runtime.seconds() <= 4) {
            drive.drive(0, 0.5, 0);
        }
        drive.stop();

    }

    private void blueBuilding() {
        moveByInches(0.6, 10, 2);

        turnDegrees(TurnDirection.COUNTER_CLOCKWISE, 90, 0.6, 1.5);

        moveByInches(0.6, 17, 2);

        turnDegrees(TurnDirection.COUNTER_CLOCKWISE, 90, 0.6, 1.5);

        moveByInches(0.6, -11, 1.5);

        liftController.toggleDragServo();

        sleep(500);

        moveByInches(0.6, 26, 1.5);

        liftController.toggleDragServo();

        sleep(500);

        runtime.reset();
        while(runtime.seconds() <= 4) {
            drive.drive(0, -0.5, 0);
        }
        drive.stop();
    }
}
