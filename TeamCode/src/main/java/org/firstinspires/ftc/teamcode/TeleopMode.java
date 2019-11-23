package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.common.FtcGamePad;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@TeleOp(name="Java: Teleop", group="Java")
public class TeleopMode extends BaseLinearOpMode implements FtcGamePad.ButtonHandler {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private FtcGamePad driverGamePad;
    private FtcGamePad operatorGamePad;
    private LiftController liftController;
    private Config config;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        config = new Config(hardwareMap.appContext);
        driverGamePad = new FtcGamePad("Driver", gamepad1, this);
        operatorGamePad = new FtcGamePad("Operator", gamepad2, this);

        Initialize(hardwareMap, driverGamePad);

        drive = new MecanumDrive(this, robot);

        liftController = new LiftController(hardwareMap, config, telemetry);

        waitForStart();
        runtime.reset();

        liftController.initLift(this);

        telemetry.log().add("(B) Drag Servo");
        telemetry.log().add("(Y) Grab Brick");
        telemetry.log().add("(A) Brick Flipper");
        telemetry.log().add("(D-Pad Down) Puts the lift down");
        telemetry.log().add("(D-Pad Left) Puts the lift at position one");
        telemetry.log().add("(D-Pad Up) Puts the lift at position two");
        telemetry.log().add("(D-Pad Right) Puts the lift at position three");

        while (opModeIsActive()) {
            drive.handle(driverGamePad);
            liftController.handle();
            driverGamePad.update();
            operatorGamePad.update();
            telemetry.update();
        }
    }

    @Override
    public void gamepadButtonEvent(FtcGamePad gamepad, int button, boolean pressed) {
        if(gamepad == operatorGamePad) {
            liftController.gamepadButtonEvent(gamepad, button, pressed);
        } else {
            drive.gamePadButtonEvent(gamepad, button, pressed);
        }
    }
}