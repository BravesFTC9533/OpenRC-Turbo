package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Autonomous(name="Nightly Test", group="nightly")
public class NightlyAutonomous extends BaseLinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private Robot robot;
    private Config config;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Robot. Hands Clear!");
        telemetry.update();

        Initialize(hardwareMap);
        initializeVuforia();

        robot = new Robot(hardwareMap);
        config = new Config(hardwareMap.appContext);

        liftController = new LiftController(hardwareMap, config, telemetry);

        drive = new MecanumDrive(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight);

        run();

    }

    private void run() {
        runtime.reset();

        while(runtime.seconds() <= 2) {
            drive.drive(0, 0.7, 0);
        }
        drive.stop();

        moveByInches(0.6, 18, 2);

        drive.drive(0, -0.5, 0);

        float distance = 0;

        while(opModeIsActive()) {
            updateVuforia();
            if(targetVisible) {
                drive.stop();
                distance = (int) Math.abs(positionX);
                break;
            }
        }

        moveByInches(0.6, -10, 1.5);
        distance += 10;

        turnDegrees(TurnDirection.COUNTER_CLOCKWISE, 180, 0.7, 2);

        moveByInches(0.6, -distance, 5);

        liftController.toggleDragServo();
    }

}
