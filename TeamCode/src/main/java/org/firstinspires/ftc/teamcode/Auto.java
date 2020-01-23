/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.controllers.ArmsController;
import org.firstinspires.ftc.teamcode.sensor.ColorSensors;

@Autonomous(name="Auto", group="Linear Opmode")
public class Auto extends BaseLinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Config.StopPosition stopPosition;
    private Config.Position startingPosition;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        super.runOpMode();

        stopPosition = config.getStopPosition();
        startingPosition = config.getPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

//        while(opModeIsActive()) {}

        switch (startingPosition) {
            case RED_BRICKS:
                redBricks();
                break;
            case BLUE_BRICKS:
                blueBricks();
                break;
            case RED_BUILDING:
                redBuilding();
                break;
            case BLUE_BUILDING:
                blueBuilding();
                break;
        }
    }

    private void redBricks() {

        // Hit the Wall
        drive.moveByInches(0.8, 28, 1.5);

        // Get flush against the wall
        drive.moveByInches(0.5, 10, 1);

        // Strafe towards the bricks
        drive.drive(0, -0.5, 0);
        while (opModeIsActive() && sensors.getSensorDistance(ColorSensors.SensorSide.RIGHT, DistanceUnit.MM) >= 55) {}
        drive.stop();

        double startInches = robot.fl.getCurrentPosition() / Robot.COUNTS_PER_INCH;
        double inchesMoved = 0;

        // Drive against the bricks until it finds the SkyStone
        while(opModeIsActive()) {
            if(sensors.isSkystone(ColorSensors.SensorSide.RIGHT)) {break;}
            drive.drive(-0.6, 0, 0);
            inchesMoved = (robot.fl.getCurrentPosition() / Robot.COUNTS_PER_INCH) - startInches;
        }
        drive.stop();

        // Delay for 0.5 seconds
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5) {}

        // Close Right Arm
        armsController.closeArm(ArmsController.ArmSide.RIGHT);

        // Delay for 0.5 seconds
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5) {}

        // Determine the stop position
        float stopTime = 3.3f;
        if(stopPosition == Config.StopPosition.WALL) {stopTime = 5f;}

        // Flip Intake
        initIntake();

        while(opModeIsActive() && intakeController.intake.getCurrentPosition() < 20) {}

        // Move towards the wall
        double start = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        drive.drive(0, 0.5, 0);
        while(opModeIsActive() && runtime.seconds() < stopTime) {
            double diff = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle - start;
            drive.drive(0, 0.5, diff / 100);
        }
        drive.stop();

        if(config.getStopPosition() == Config.StopPosition.BRIDGE) {
            drive.moveByInches(1, -63 - (int) inchesMoved, 2);
        } else {
            drive.moveByInches(1, -42 - (int) inchesMoved, 2);
        }

        armsController.openArm(ArmsController.ArmSide.RIGHT);

        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.25) {}

        if(config.getStopPosition() == Config.StopPosition.BRIDGE) {
            drive.moveByInches(1, 23, 1.5);
        } else {
            drive.moveByInches(1, 18, 1.5);
        }

        if(config.getStopPosition() == Config.StopPosition.BRIDGE) {
            runtime.reset();
            drive.drive(0, -0.5, 0);
            while (opModeIsActive() && runtime.seconds() < 2.5) {}
            drive.stop();
        } else {
            runtime.reset();
            drive.drive(0, 0.5, 0);
            while(opModeIsActive() && runtime.seconds() < 1) {}
            drive.stop();
        }
    }

    private void redBuilding() {

        // Hit the wall
        drive.moveByInches(0.8, 24, 1.5);

        runtime.reset();
        drive.drive(0, 0.5, 0);
        while(opModeIsActive() && runtime.seconds() < 3) {}
        drive.stop();

        // Get flush against wall
        drive.moveByInches(0.8, 20, 1);

        // Back up
        drive.moveByInches(0.6, -10, 1);

        // Lower the arm
        armsController.closeArm(ArmsController.ArmSide.LEFT);

        // Delay 0.5s
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5) {}

        // Turn and back-up
        drive.drive(-1, 0.5, 0.7);

        // Wait five seconds
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 1.5) {}
        drive.stop();

        // Go forward

        drive.drive(0.8, 0, 0);

        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 3) {}
        drive.stop();

    }

    private void blueBricks() {
        // Hit the Wall
        drive.moveByInches(0.8, 28, 1.5);

        // Get flush against the wall
        drive.moveByInches(0.5, 10, 1);

        // Strafe towards the bricks
        double start = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        drive.drive(0, 0.45, 0);
        while(opModeIsActive() && sensors.getSensorDistance(ColorSensors.SensorSide.LEFT, DistanceUnit.MM) >= 57) {
            double diff = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle - start;
            drive.drive(0, 0.45, diff / 100);
        }
        drive.stop();

        // Hit the wall
        drive.moveByInches(0.8, 15, 1.5);

        drive.drive(0, 0.45, 0);
        while(opModeIsActive() && sensors.getSensorDistance(ColorSensors.SensorSide.LEFT, DistanceUnit.MM) >= 55) {
            double diff = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle - start;
            drive.drive(0, 0.45, diff / 100);
        }
        drive.stop();

        double startInches = robot.fl.getCurrentPosition() / Robot.COUNTS_PER_INCH;
        double inchesMoved = 0;

        // Drive against the bricks until it finds the SkyStone
        while(opModeIsActive()) {
            if(sensors.isSkystone(ColorSensors.SensorSide.LEFT)) {break;}
            drive.drive(-0.6, 0, 0);
            inchesMoved = robot.fl.getCurrentPosition() / Robot.COUNTS_PER_INCH - startInches;
        }
        drive.stop();

        runtime.reset();

        drive.drive(0, 0.45, 0);
        while(opModeIsActive() && runtime.seconds() < 0.35) {}
        drive.stop();

        // Delay for 0.5 seconds
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5) {}

        // Close Right Arm
        armsController.closeArm(ArmsController.ArmSide.LEFT);

        // Delay for 0.5 seconds
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5) {}

        // Determine the stop position
        float stopTime = 2.95f;
        if(stopPosition == Config.StopPosition.WALL) {stopTime = 6f;}

        // Flip Intake
        initIntake();

        while(opModeIsActive() && intakeController.intake.getCurrentPosition() < 20) {}

        // Move towards the wall
        start = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        drive.drive(0, -0.5, 0);
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            double diff = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle - start;
            drive.drive(-0.3, -0.5, diff / 100);
        }
        drive.stop();

        drive.drive(0, -0.5, 0);
        while(opModeIsActive() && runtime.seconds() < stopTime) {
            double diff = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle - start;
            drive.drive(-0.15, -0.5, diff / 100);
        }
        drive.stop();

        if(config.getStopPosition() == Config.StopPosition.BRIDGE) {
            drive.moveByInches(1, -73 - (int) inchesMoved, 2);
        } else {
            drive.moveByInches(1, -82 - (int) inchesMoved, 2);
        }

        armsController.openArm(ArmsController.ArmSide.LEFT);

        if(config.getStopPosition() == Config.StopPosition.BRIDGE) {
            drive.moveByInches(1, 23, 1.5);
        } else {
            drive.moveByInches(1, 20, 1.5);
        }

        if(config.getStopPosition() == Config.StopPosition.BRIDGE) {
            runtime.reset();
            drive.drive(0, 0.5, 0);
            while (opModeIsActive() && runtime.seconds() < 1.5) {}
            drive.stop();
        }
    }

    private void blueBuilding() {

    }
}
