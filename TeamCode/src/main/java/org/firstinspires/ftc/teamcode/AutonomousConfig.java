package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.common.SimpleMenu;


@Autonomous(name="Configuration", group="Config")
public class AutonomousConfig extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public static SimpleMenu menu = new SimpleMenu("Autonomous Menu");

    @Override
    public void runOpMode() {
        Config config = new Config(hardwareMap.appContext);

        menu.clearOptions();

        menu.addOption("Position", Config.Position.class, config.getPosition());
        menu.addOption("Max Lift Ticks", 10000, 0, 1, config.getMaxLiftTicks());
        menu.addOption("Max Servo Position", 1, 0, 0.1, config.getMaxLiftTicks());
        menu.setGamepad(gamepad1);
        menu.setTelemetry(telemetry);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            menu.displayMenu();

            switch (menu.getCurrentChoiceOf("Position")) {
                case "BLUE_BRICKS":
                    config.setPosition(Config.Position.BLUE_BRICKS);
                    break;
                case "BLUE_BUILDING":
                    config.setPosition(Config.Position.BLUE_BUILDING);
                    break;
                case "RED_BRICKS":
                    config.setPosition(Config.Position.RED_BRICKS);
                    break;
                case "RED_BUILDING":
                    config.setPosition(Config.Position.RED_BUILDING);
                    break;
            }

            config.setMaxLiftTicks((int) Double.parseDouble(menu.getCurrentChoiceOf("Max Lift Ticks")));
            config.setMaxServoPosition((float) Double.parseDouble(menu.getCurrentChoiceOf("Max Servo Position")));

            sleep(50);
        }

        config.save();
    }
}
