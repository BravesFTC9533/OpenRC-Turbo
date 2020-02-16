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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Auto;
import org.firstinspires.ftc.teamcode.controllers.ArmsController;
import org.firstinspires.ftc.teamcode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.drive.Drive;
import org.firstinspires.ftc.teamcode.drive.MechDrive;
import org.firstinspires.ftc.teamcode.sensor.ColorSensors;

import java.util.ArrayList;
import java.util.List;

public class BaseLinearOpMode extends LinearOpMode {

    protected Robot robot;
    protected Config config;
    protected Drive drive;
    protected ColorSensors sensors;
    protected ArmsController armsController;
    protected LiftController liftController;
    protected IntakeController intakeController;

    @Override
    public void runOpMode() {

    }

    protected void Initialize() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        robot = new Robot(hardwareMap, telemetry);

        robot.imu.initialize(robot.params);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(!isStopRequested() && !robot.imu.isGyroCalibrated() && timer.seconds() < 1.75) {
            sleep(50);
            idle();
        }

        config = new Config(hardwareMap.appContext);
        drive = new MechDrive(robot, this);
        sensors = new ColorSensors(hardwareMap);
        liftController = new LiftController(this, hardwareMap);
        intakeController = new IntakeController(this, hardwareMap);
        armsController = new ArmsController(hardwareMap);

        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    public void initIntake() {
        intakeController = new IntakeController(this, hardwareMap);
        intakeController.init();
    }

    public void initLift() {
        armsController.init();
    }

    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/Skystone.tflite"; //For OpenRC, loaded from internal storage to reduce APK size
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AXbBhc3/////AAABmYrvTPoIKkxfmXR8QBJ3kFhjEocjRRhMOdtB3FD5YzoKJDKhgPBoVU6Q0qODLRLeUmOxRs/FN8m875DQGgty/sfg3wzFJLbjUPfMjxctY4hasa8ydNdgy1Bo4IvOnoN80gyAkhzAw2yyQgcjg4su/5nBPlVxx/JXAHMs7il8Lf2P7ZGfvZiGC+5DkVTNwPdWgXUyq5zixPy9052+jy93106KutP6JCfCrPWNZrJvKZhQDqOL7om/5gKUbGI57T8IXq/o4n6yL3ha4ADMjG2tiF54yOOfh8E96Mn3PgsuLQqoQYps/gAf3mdK0wV+qW7o0lTiZ0qtQzopxAO5Jp8nUwc1Bl5s018gLNJXouXjkgkQ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    protected void startVuforia() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
    }

    private List<Recognition> updatedRecognitions;

    protected void updateTFOD() {
        if(!isStopRequested() && tfod != null) {
            if (!isStopRequested() && tfod != null) {
                updatedRecognitions = tfod.getUpdatedRecognitions();
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                if (updatedRecognitions != null && !isStopRequested()) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    telemetry.update();
                }
            }
        }

        if(isStopRequested()) {
            tfod.shutdown();
        }
    }

    protected double left = 0;

    public void deactivate() {
        if(tfod == null) return;
        tfod.shutdown();
    }

    public boolean detectSkystone() {
        if(!isStopRequested() && tfod != null) {
            updateTFOD();
        }

        if(!isStopRequested() && tfod != null) {
            if(updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if (!isStopRequested() && recognition.getLabel().equals("Skystone")) {
                        if(recognition.getLeft() <= 25) {
                            return false;
                        }
                        left = recognition.getLeft();
                    }
                }
            }
        }
        return false;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


}
