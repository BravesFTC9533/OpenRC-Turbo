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

import org.firstinspires.ftc.teamcode.common.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.drive.Drive;
import org.firstinspires.ftc.teamcode.drive.MechDrive;

@Autonomous(name="Auto", group="Linear Opmode")
public class Auto extends BaseLinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Config.StopPosition stopPosition;
    private Config.Position startingPosition;

    @Override
    public void runOpMode() {
        super.Initialize();
        super.startVuforia();

        armsController.init();
        liftController.init();

        stopPosition = config.getStopPosition();
        startingPosition = config.getPosition();

        while(!isStopRequested() && !isStarted()) {
            skystonePosition = detectSkystone(startingPosition);

            telemetry.addData("Skystone Position", skystonePosition);
            telemetry.addData("Position", left);
            telemetry.update();

            if(runtime.seconds() >= 0.25 && !isStarted()) {
                runtime.reset();
            }
        }

        waitForStart();
        runtime.reset();

        super.deactivate();

        switch (startingPosition) {
            case RED_BRICKS:
                break;
            case BLUE_BRICKS:
                blueBricks();
                break;
        }
        runAuto();
    }

    private SkystonePosition skystonePosition = SkystonePosition.CENTER;
    public enum SkystonePosition {
        LEFT, CENTER, RIGHT
    }

    private SkystoneOffset skystoneOffset = SkystoneOffset.CENTER;
    private enum SkystoneOffset {
        FAR, CENTER, NEAR
    }

    private Drive.TurnDirection turnDirection = Drive.TurnDirection.COUNTER_CLOCKWISE;
    private int brickWidth = 8; // inches
    private float farBrickOffset = 5.65f;

    private float brickDistFromRobot = 5f;
    private float strafeDistFromBricks = 11f;

    private float baseDistanceFar = -53f;

    private void blueBricks() {
        turnDirection = Drive.TurnDirection.CLOCKWISE;
        brickDistFromRobot = -brickDistFromRobot;
        strafeDistFromBricks = -strafeDistFromBricks;
    }

    private void runAuto() {
        // Move Towards Bricks
        drive.moveByInches(1, 24, 1.5);

        super.initIntake();

        // Turn 90 degs
        drive.turnDegrees(1, turnDirection, 90, 1.5);

        // Figure Position
        if(skystonePosition == SkystonePosition.LEFT) {
            skystoneOffset = startingPosition == Config.Position.RED_BRICKS ? SkystoneOffset.FAR: SkystoneOffset.NEAR;
        } else if(skystonePosition == SkystonePosition.CENTER) {
            skystoneOffset = SkystoneOffset.CENTER;
        } else if (skystonePosition == SkystonePosition.RIGHT) {
            skystoneOffset = startingPosition == Config.Position.RED_BRICKS ? SkystoneOffset.NEAR: SkystoneOffset.FAR;
        }

        // Figure Position
        if(skystoneOffset == SkystoneOffset.FAR) {
            telemetry.addData("Status", "Left");
            telemetry.update();
            drive.moveByInches(1, farBrickOffset, 1);
        } else if(skystoneOffset == SkystoneOffset.CENTER) {
            telemetry.addData("Status", "Center");
            telemetry.update();
            drive.moveByInches(1, farBrickOffset - brickWidth, 1);
        } else if (skystoneOffset == SkystoneOffset.NEAR) {
            telemetry.addData("Status", "Right");
            telemetry.update();
            drive.moveByInches(1, farBrickOffset - (brickWidth * 2), 1);
        }

        // Move Into From Bricks
        ((MechDrive) drive).strafe(-0.5, brickDistFromRobot, 2);

        // Delay 0.5s
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5) {}

        // Grab Brick
        if(startingPosition == Config.Position.RED_BRICKS) {
            armsController.rightArm.setPosition(-1);
        } else {
            armsController.leftArm.setPosition(-1);
        }
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5) {}

        // Move Away From Bricks
        ((MechDrive) drive).strafe(0.5, strafeDistFromBricks, 2);

        // Wait 0.5s
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.75) {}

        // Figure move distance
        float moveDistance = baseDistanceFar;
        if(skystoneOffset == SkystoneOffset.CENTER) moveDistance += brickWidth;
        if(skystoneOffset == SkystoneOffset.NEAR) moveDistance += (brickWidth * 2);

        // Move across the tape
        drive.moveByInches(1, moveDistance, 2.5);

        // Drop Brick
        if(startingPosition == Config.Position.RED_BRICKS) {
            armsController.rightArm.setPosition(1);
        } else {
            armsController.leftArm.setPosition(1);
        }

        // Wait 0.5s
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5) {}

        // Figure move distance
        moveDistance = -moveDistance + (brickWidth * 3);

        // Move back across the tape
        drive.moveByInches(1, moveDistance, 2.5);

        // Strafe into bricks
        ((MechDrive) drive).strafe(-0.45, strafeDistFromBricks, 2.5);

        // Grab Brick
        if(startingPosition == Config.Position.RED_BRICKS) {
            armsController.rightArm.setPosition(-1);
        } else {
            armsController.leftArm.setPosition(-1);
        }
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5) {}

        // Move Away From Bricks
        ((MechDrive) drive).strafe(0.5, strafeDistFromBricks, 2);

        // Move back across the tape
        drive.moveByInches(1, -moveDistance, 2.5);

        // Drop Brick
        if(startingPosition == Config.Position.RED_BRICKS) {
            armsController.rightArm.setPosition(1);
        } else {
            armsController.leftArm.setPosition(1);
        }

        // Park on tape
        drive.moveByInches(1, 10, 1);
    }

    private void grabStageBricks() {

    }

    private void redBuilding() {

    }

    private void blueBuilding() {

    }

}
