package org.firstinspires.ftc.teamcode.adultbot.hardware.motors;

import org.firstinspires.ftc.teamcode.adultbot.util.ImmutableList;
import org.firstinspires.ftc.teamcode.adultbot.util.units.Velocity;



public class FourMotors extends NMotors {
    public FourMotors(Motor frontLeftMotor, Motor frontRightMotor, Motor backLeftMotor, Motor backRightMotor, boolean useSpeedMode, Velocity maxRobotSpeed) {
        super(ImmutableList.of(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor), useSpeedMode, maxRobotSpeed);
    }

    public void runMotorsNormalized(double flValue, double frValue, double blValue, double brValue) {
        runNormalized(ImmutableList.of(flValue, frValue, blValue, brValue));
    }

    public void runMotors(double flValue, double frValue, double blValue, double brValue) {
        run(ImmutableList.of(flValue, frValue, blValue, brValue));
    }
}
