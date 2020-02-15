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

@Autonomous(name="Auto", group="Linear Opmode")
public class Auto extends BaseLinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Config.StopPosition stopPosition;
    private Config.Position startingPosition;

    @Override
    public void runOpMode() {
        super.runOpMode();

        stopPosition = config.getStopPosition();
        startingPosition = config.getPosition();

        // Determine Skystone Position
        while(opModeIsActive() && !isStarted()) {


            if(runtime.seconds() >= 2.0) {

                //look for skystone

                runtime.reset();
            }

//            double startTime = System.nanoTime();
//            while(opModeIsActive() && !isStarted()) {
//                double currentTime = System.nanoTime();
//                if((currentTime - startTime) >= 1 && !isStarted()) {
//                    break;
//                }
//            }
        }

        waitForStart();
        runtime.reset();
        
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

    private enum SkystonePosition {
        LEFT, CENTER, RIGHT
    }
    private SkystonePosition skystonePosition;

    private void redBricks() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(opModeIsActive()) {
            ifSkyStone();
            telemetry.addData("Left", super.left);
        }

//        switch (skystonePosition) {
//            case LEFT:
//                break;
//            case RIGHT:
//                break;
//            case CENTER:
//                break;
//        }
    }

    private void blueBricks() {

    }

    private void redBuilding() {

    }

    private void blueBuilding() {

    }

}
