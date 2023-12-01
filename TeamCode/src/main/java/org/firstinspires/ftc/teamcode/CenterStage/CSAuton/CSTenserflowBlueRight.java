/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.CenterStage.CSAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.CenterStage.CSRobot.CSrobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */

@Autonomous
public class CSTenserflowBlueRight extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.

    //Use Default model if required
    // private static final String TFOD_MODEL_ASSET = "CenterStage.tflite";

    //custom Model
    private static final String TFOD_MODEL_ASSET = "BlueLeftNew.tflite";
    private static final String[] LABELS = {"BPLeft"};

    /* private static final String[] LABELS = {
            "Pixel",
    };

   */
    private static final boolean RP_VALUE = false;
    private static final boolean RSS_VALUE = false;
    private static final boolean RS_VALUE = false;
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    // private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/CenterStage.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static String spikeDetected = "RIGHT";
    //Aril Tag Detection/Movement
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private final ElapsedTime timer = new ElapsedTime();
    public CSrobot robot;
    //Left Spike
    //X-Axis 39 130
    double leftXLowerBound = 35.0;
    double leftXUpperBound = 195.0;
    //Object Detection Variables  Calibrated for x and Y axis.
    // These values should be calibrated by using the camera set on the robot
     //Y-Axis 281 225
    double leftYLowerBound = 200.0;
    double leftYUpperBound = 300.0;
    //Center Spike
    //X-Axis 284 409 459
    double centerXLowerBound = 255.0;
    double centerXUpperBound = 475.0;
    //Y-Axis 221 229 232
    double centerYLowerBound = 210.0;
    double centerYUpperBound = 245.0;
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private boolean targetFound = false;
    private double drive = 0.0;        // Desired forward power/speed (-1 to +1)
    private double strafe = 0.0;        // Desired strafe power/speed (-1 to +1)
    private double turn = 0.0;        // Desired turning power/speed (-1 to +1)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    @Override
    public void runOpMode() {

        robot = new CSrobot(this);
        robot.arm.SetMotorPower(0.0);
        initDoubleVision();
        //initiallize visionPortal
        if (opModeInInit()) {
            visionPortal.setProcessorEnabled(tfod, true);
            visionPortal.setProcessorEnabled(aprilTag, false);
        }
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        /* This OpMode loops continuously, allowing the user to switch between
         AprilTag and TensorFlow Object Detection (TFOD) image processors.
         */

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                //telemetryTfod();
                if (visionPortal.getProcessorEnabled(tfod)) {
                    telemetry.addLine("Tensor flow enabled");
                    telemetry.addLine();
                    telemetryTfod();
                } else {
                    telemetry.addLine("No tensor flow object detected");
                }

                // Push telemetry to the Driver Station.
                telemetry.update();


                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }   // end runOpMode()


    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        // Step through the list of recognitions and display info for each one.
        if (currentRecognitions.size() != 0) {
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;

                //Display Information on screen
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                telemetry.addData("- Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));
                telemetry.update();
                if (recognition.getLabel().equals("BPLeft")) {
                    if (((x >= leftXLowerBound) && (x <= leftXUpperBound)) && ((y >= leftYLowerBound) && (y <= leftYUpperBound))) {
                        telemetry.addData("Team Prop Location AXIS", "LEFT AXIS");
                        telemetry.update();
                        spikeDetected = "LEFT";
                        leftFunction();
                        telemetry.update();
                        break;
                    } else if (((x >= centerXLowerBound) && (x <= centerXUpperBound)) && ((y >= centerYLowerBound) && (y <= centerYUpperBound))) {
                        telemetry.addData("Team Prop Location AXIS ", "CENTER AXIS ");
                        telemetry.update();
                        spikeDetected = "CENTER";
                        centerFunction();
                        telemetry.update();
                        break;
                    }
                } else {
                    telemetry.addData("Team Prop Location AXIS ", "RIGHT AXIS");
                    telemetry.update();
                    spikeDetected = "RIGHT";
                   rightFunction();
                    telemetry.update();
                    break;
                }


            }   // end method telemetryTfod()
            timer.reset();
        } else {

            telemetry.addData("No Object detected yet", "Trying Again");
            if (timer.seconds() >= 7) {
                telemetry.addData("Team Prop Location AXIS ", "RIGHT AXIS");
                telemetry.update();
                spikeDetected = "RIGHT";
                rightFunction();
                //centerFunction();
                // leftFunction();
                telemetry.update();
            }
        }

        //set position

    }   // end class

    private void rightFunction() {

        //Strafe to center
        robot.Drive.MoveRobotToPositionStrafe(0.5, 18);
        sleep(700);

        //move robot to TeamProp
        robot.Drive.MoveRobotToPosition(0.5, 19);
        sleep(700);

        //Drop Pixel
        robot.clawLow.setClawPosition(0.5);
        sleep(700);

        robot.Drive.MoveRobotToPosition(-0.5, -12);
        sleep(700);

        /*below must be commented if we are not parking*/
        /*----------------parking start---------------*/
        //Allign to position
      /*  robot.Drive.pointTurn(-180, 0.5);
        sleep(700);


        //Strafe to last qudrant
        robot.Drive.MoveRobotToPositionStrafe(-0.5, -10);
        sleep(700);

        //Move to parking
        robot.Drive.MoveRobotToPosition(0.5, 85);
        sleep(700);

        /*----------------parking end---------------*/

        sleep(25000);


    }

    private void centerFunction() {

        robot.Drive.MoveRobotToPositionStrafe(0.5, 20);
         sleep(700);

        //move robot to TeamProp
        robot.Drive.MoveRobotToPosition(0.5, 24);
        sleep(700);

        robot.Drive.pointTurn(-45, -0.5);
        sleep(700);

        robot.Drive.MoveRobotToPosition(0.5, 7);
        sleep(700);

        //Drop Pixel
        robot.clawLow.setClawPosition(0.5);
        sleep(700);


        //move back
        robot.Drive.MoveRobotToPosition(-0.5, -20);
        sleep(700);


        /*below must be commented if we are not parking*/
        /*----------------parking start---------------*/
        //Allign to position
       /* robot.Drive.pointTurn(-180, -0.5);
        sleep(700);


        //Strafe to last qudrant
        robot.Drive.MoveRobotToPositionStrafe(-0.5, -10);
        sleep(700);

        //Move to parking
        robot.Drive.MoveRobotToPosition(0.5, 85);
        sleep(700);

        */

        /*----------------parking end---------------*/


        sleep(25000);
    }

    private void leftFunction() {

        //Strafe to location
        robot.Drive.MoveRobotToPositionStrafe(0.5, 20);
        sleep(700);

        //Move forward and allign with team prop
        robot.Drive.MoveRobotToPosition(0.5, 19);
        sleep(700);

        //Turn towards the team prop
        robot.Drive.pointTurn(-130, -0.5);
        sleep(700);

        //Move robot to TeamProp
        robot.Drive.MoveRobotToPosition(0.3, 18);
        sleep(700);

        //Drop Pixel
        robot.clawLow.setClawPosition(0.5);
        sleep(700);

        //move back
        robot.Drive.MoveRobotToPosition(-0.5, -19);
        sleep(700);

        /*below must be commented if we are not parking*/
        /*----------------parking start---------------*/
        //Allign to position
       /* robot.Drive.pointTurn(-40, -0.5);
        sleep(700);


        //Strafe to last qudrant
        robot.Drive.MoveRobotToPositionStrafe(-0.5, -10);
        sleep(700);

        //Move to parking
        robot.Drive.MoveRobotToPosition(0.5, 85);
        sleep(700);

        /*----------------parking end---------------*/

        sleep(25000);
    }


    /**
     * Initialize AprilTag and TFOD.
     */
    private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);


        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------

        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                // Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)
                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setModelAspectRatio(16.0 / 9.0)
                .build();
        //tfod.setZoom(1.2);
        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.60f);

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .enableLiveView(true)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }   // end initDoubleVision()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag(int tagID) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        targetFound = false;
        desiredTag = null;

        if (USE_WEBCAM) {
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        }

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    //  Check to see if we want to track towards this tag.
                    if ((tagID < 0) || (detection.id == tagID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop

            // Tell the robot what we see, and what to do.
            if (targetFound) {
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>", "Tag is Not detected\n");
            }

            // If we have found the desired target, Drive to target Automatically .
            if (targetFound) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;
                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {
                telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            // Move robot using AprilTag Data and and perfectly aligned the robot
           // moveRobot(drive, strafe, turn);
            robot.Drive.MoveRobotToPosition(.5,(desiredTag.ftcPose.range)/2);
            robot.Drive.pointTurn(desiredTag.ftcPose.bearing,.5);
            //sleep(700);
        if(tagID==3) {
            robot.Drive.MoveRobotToPosition(.5,(desiredTag.ftcPose.range)/2);
            robot.Drive.pointTurn(desiredTag.ftcPose.bearing,.5);
            robot.Drive.MoveRobotToPositionStrafe(.5, desiredTag.ftcPose.yaw);
        }else if(tagID==2) {
            robot.Drive.MoveRobotToPosition(.5,(desiredTag.ftcPose.range)/2);
            robot.Drive.pointTurn(desiredTag.ftcPose.bearing,.5);
            robot.Drive.MoveRobotToPositionStrafe(.5, desiredTag.ftcPose.yaw+6);
        }else if(tagID==1){
            robot.Drive.MoveRobotToPosition(.5,(desiredTag.ftcPose.range)/2);
            robot.Drive.pointTurn(desiredTag.ftcPose.bearing,.5);
        }
           sleep(10);



       // }//end whileloop

        //Revert motor direction
      //  robot.Drive.setMotorDirection(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);


    }   // end method telemetryAprilTag()


    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }





        //new addition
        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

}