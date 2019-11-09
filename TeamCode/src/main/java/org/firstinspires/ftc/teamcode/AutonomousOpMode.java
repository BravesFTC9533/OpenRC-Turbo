package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.controllers.LiftController;

@Autonomous(name = "Java: Autonomous", group = "Concept")
public class AutonomousOpMode extends BaseLinearOpMode {

    private static final double LEFT_MIN = 22.0;
    private static final double LEFT_MAX = 14.0;

    private static final double RIGHT_MAX = 9.9;
    private static final double RIGHT_MIN = 0.0;

    private ElapsedTime runtime = new ElapsedTime();

    private LiftController liftController;
    private Config config;

    private Config.Position startingPosition;
    private BrickPosition brickPosition = BrickPosition.CENTER;

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
        moveByInches(0.5, 4);
        turnDegrees(TurnDirection.COUNTER_CLOCKWISE, 90, 0.5);

        moveByInches(0.6, 20);

        turnDegrees(TurnDirection.CLOCKWISE, 90, 0.5);

        moveByInches(0.6, 8);

        runtime.reset();

        while(opModeIsActive() && !isTargetFound && runtime.seconds() <= 12) {
            updateVuforia();
            if(targetVisible) {
                telemetry.addLine("Did Stuff");
                if(positionY <= LEFT_MAX && positionY >= LEFT_MIN) {
                    brickPosition = BrickPosition.LEFT;
                } else if(positionY <= RIGHT_MAX && positionY >= RIGHT_MIN) {
                    brickPosition = BrickPosition.RIGHT;
                }

                isTargetFound = true;
            }
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", firstAngle, secondAngle, thirdAngle);
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    positionX / mmPerInch, positionY / mmPerInch, positionZ / mmPerInch);
            telemetry.update();
        }

        if(isTargetFound) {
            telemetry.addLine("Yay");
            switch (brickPosition) {
                case CENTER:
                    turnDegrees(TurnDirection.CLOCKWISE, 90, 0.5);

                    moveByInches(0.6, 3);

                    turnDegrees(TurnDirection.COUNTER_CLOCKWISE, 90, 0.5);

                    moveByInches(0.6, 3);

                    turnDegrees(TurnDirection.CLOCKWISE, 180, 0.6);

                    moveByInches(0.6, 5);

                    liftController.toggleDragServo();

                    moveByInches(0.6, 8);

                    turnDegrees(TurnDirection.COUNTER_CLOCKWISE, 90, 0.5);

                    moveByInches(0.6, 90);

                    turnDegrees(TurnDirection.CLOCKWISE, 90, 0.5);

                    liftController.toggleDragServo();

                    moveByInches(0.6, -3);

                    moveByInches(0.6, 3);

                    turnDegrees(TurnDirection.CLOCKWISE, 90, 180);

                    moveByInches(0.6, 30);
                    break;
                case LEFT:
                    moveByInches(0.6, 3);

                    turnDegrees(TurnDirection.CLOCKWISE, 180, 0.6);

                    moveByInches(0.6, 5);

                    liftController.toggleDragServo();

                    moveByInches(0.6, 8);

                    turnDegrees(TurnDirection.COUNTER_CLOCKWISE, 90, 0.5);

                    moveByInches(0.6, 90);

                    turnDegrees(TurnDirection.CLOCKWISE, 90, 0.5);

                    liftController.toggleDragServo();

                    moveByInches(0.6, -3);

                    moveByInches(0.6, 3);

                    turnDegrees(TurnDirection.CLOCKWISE, 90, 180);

                    moveByInches(0.6, 30);
                    break;
                case RIGHT:
                    turnDegrees(TurnDirection.CLOCKWISE, 90, 0.5);

                    moveByInches(0.6, 6);

                    turnDegrees(TurnDirection.COUNTER_CLOCKWISE, 90, 0.5);

                    moveByInches(0.6, 3);

                    turnDegrees(TurnDirection.CLOCKWISE, 180, 0.6);

                    moveByInches(0.6, 5);

                    liftController.toggleDragServo();

                    moveByInches(0.6, 8);

                    turnDegrees(TurnDirection.COUNTER_CLOCKWISE, 90, 0.5);

                    moveByInches(0.6, 90);

                    turnDegrees(TurnDirection.CLOCKWISE, 90, 0.5);

                    liftController.toggleDragServo();

                    moveByInches(0.6, -3);

                    moveByInches(0.6, 3);

                    turnDegrees(TurnDirection.CLOCKWISE, 90, 180);

                    moveByInches(0.6, 30);
                    break;
            }
        } else {
            telemetry.addLine("Could not find the target");
            telemetry.update();

            moveByInches(0.6, 3);

            turnDegrees(TurnDirection.CLOCKWISE, 180, 0.6);

            moveByInches(0.6, 5);

            liftController.toggleDragServo();

            moveByInches(0.6, 8);

            turnDegrees(TurnDirection.COUNTER_CLOCKWISE, 90, 0.5);

            moveByInches(0.6, 90);

            turnDegrees(TurnDirection.CLOCKWISE, 90, 0.5);

            liftController.toggleDragServo();

            moveByInches(0.6, -3);

            moveByInches(0.6, 3);

            turnDegrees(TurnDirection.CLOCKWISE, 90, 180);

            moveByInches(0.6, 30);
        }
    }

    private void blueBricks() {

    }

    private void blueBuilding() {

    }

    private void redBuilding() {

    }
}
