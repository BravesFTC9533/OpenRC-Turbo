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

package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TFod;
import org.firstinspires.ftc.teamcode.controllers.ArmsController;
import org.firstinspires.ftc.teamcode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.drive.Drive;
import org.firstinspires.ftc.teamcode.drive.MechDrive;
import org.firstinspires.ftc.teamcode.sensor.ColorSensors;

public class BaseLinearOpMode extends LinearOpMode {

    protected Robot robot;
    protected Config config;
    protected Drive drive;
    protected ColorSensors sensors;
    protected ArmsController armsController;
    protected LiftController liftController;
    protected IntakeController intakeController;

    protected TFod tfod;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);

        robot.imu.initialize(robot.params);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(!isStopRequested() && !robot.imu.isGyroCalibrated() && timer.seconds() < 1.75) {
            sleep(50);
            idle();
        }

        tfod = new TFod();

        config = new Config(hardwareMap.appContext);
        drive = new MechDrive(robot, this);
        sensors = new ColorSensors(hardwareMap);
        liftController = new LiftController(this, hardwareMap);
        intakeController = new IntakeController(this, hardwareMap);
        armsController = new ArmsController(hardwareMap);

        telemetry.addData("Status", "Robot is Initialized.");
        telemetry.update();

        waitForStart();

        armsController.init();
        liftController.init();
    }

    public void initIntake() {
        intakeController = new IntakeController(this, hardwareMap);
        intakeController.init();
    }

    public void initLift() {
        armsController.init();
    }
}
