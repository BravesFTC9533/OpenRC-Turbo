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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.ArmsController;
import org.firstinspires.ftc.teamcode.common.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.drive.Drive;
import org.firstinspires.ftc.teamcode.drive.MechDrive;
import org.firstinspires.ftc.teamcode.sensor.ColorSensors;

@Autonomous(name="Auto", group="Linear Opmode")
public class Auto extends BaseLinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Config.ParkPosition parkPosition;
    private Config.Position startingPosition;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        super.runOpMode();

        parkPosition = config.getParkPosition();
        startingPosition = config.getPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

//        switch (startingPosition) {
//            case RED_BRICKS:
//                redBricks();
//                break;
//            case BLUE_BRICKS:
//                blueBricks();
//                break;
//            case RED_BUILDING:
//                redBuilding();
//                break;
//            case BLUE_BUILDING:
//                blueBuilding();
//                break;
//        }
    }

    private void redBricks() {
        // Move off the wall.
        drive.moveByInches(0.8, 20, 1.75);

        // Turn 90 degrees towards the wall.
        drive.turnDegrees(1, Drive.TurnDirection.COUNTER_CLOCKWISE, 90, 1.5);

        // Move backwards.
        drive.moveByInches(0.6, 30, 2);

        // Move towards bricks.
        drive.drive(0, 0.5, 0);

        // Wait until we are 70mm away from the bricks.
        while(opModeIsActive()) {
            if(sensors.getSensorDistance(ColorSensors.SensorSide.FRONT, DistanceUnit.MM) <= 70) {
                drive.stop();
                break;
            }
        }

        // Hit against the wall to straighten out.
        drive.moveByInches(0.5, 5, 1);

        // Move forward slowly.
        drive.drive(0.6, 0, 0);

        // Check for the yellow brick.
        while(opModeIsActive()) {
            if(sensors.isSkystone(ColorSensors.SensorSide.FRONT)) {
                drive.stop();
                break;
            }
        }

        // Grab the yellow brick with front arm.
        armsController.closeArm(ArmsController.ArmSide.FRONT);
    }

    private void redBuilding() {

    }

    private void blueBricks() {

    }

    private void blueBuilding() {

    }
}
