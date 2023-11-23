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

package org.firstinspires.ftc.teamcode.CenterStage.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Disabled
@Autonomous
public class RedRight extends LinearOpMode {
    //testing Git for NOv22Branch

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.

    //Default model
   // private static final String TFOD_MODEL_ASSET = "CenterStage.tflite";

    //custom Model
     private static final String TFOD_MODEL_ASSET = "PPRed.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    // private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/CenterStage.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static  String spikeDetected = "LEFT";

    /* private static final String[] LABELS = {
            "Pixel",
    };

   */

    private static final String[] LABELS = {
            "RP","RS","RSS"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    //Left Spike
//X-Axis
    double leftXLowerBound = 480.0;
    double leftXUpperBound = 570.0;
    //Object Detection Variables  Calibrated for x and Y axis.
// These values should be calibrated by using the camera set on the robot
//Y-Axis
    double leftYLowerBound = 230.0;
    double leftYUpperBound = 330.0;
    //Center Spike
//X-Axis
    double centerXLowerBound = 56.0;
    double centerXUpperBound = 361.0;
    //X-Axis
    double centerYLowerBound = 226.0;
    double centerYUpperBound = 251.0;

    private static final boolean RP_VALUE = false;
    private static final boolean RSS_VALUE = false;
    private static final boolean RS_VALUE = false;
    public CSrobot robot;
    private final ElapsedTime timer =new ElapsedTime();

    @Override
    public void runOpMode() {

        robot = new CSrobot(this);
        robot.arm.SetMotorPower(0.0);
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        //waitForStart();


            while (!isStarted() && !isStopRequested()) {
                telemetryTfod();
                telemetry.update();
                sleep(20);
            }

        waitForStart();
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
        if (spikeDetected == "LEFT") {
            //Default path is LEFT

            telemetry.addLine("Left  spot");
            leftFunction();
        } else if (spikeDetected == "CENTER") {

            telemetry.addLine("Middle  spot");
            centerFunction();
        } else if (spikeDetected == "RIGHT") {

            telemetry.addLine("Right  spot");
            rightFunction();
        }
        else {

            telemetry.addLine("No spot");
        }


    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

          // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);


        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.65f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        // Step through the list of recognitions and display info for each one.
        if(currentRecognitions!=null){
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;


            //telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.addData("- Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));
            telemetry.update();
            if (recognition.getLabel().equals("RP")) {
                if (((x >= leftXLowerBound) && (x <= leftXUpperBound)) && ((y >= leftYLowerBound) && (y <= leftYUpperBound))) {
                    telemetry.addData("Team Prop Location AXIS", "RIGHT AXIS");
                    telemetry.update();
                    spikeDetected = "RIGHT";
                   // rightFunction();
                    telemetry.update();
                    break;
                } else if (((x >= centerXLowerBound) && (x <= centerXUpperBound)) && ((y >= centerYLowerBound) && (y <= centerYUpperBound))) {
                    telemetry.addData("Team Prop Location AXIS ", "CENTER AXIS ");
                    telemetry.update();
                    spikeDetected = "CENTER";
                   // centerFunction();
                    telemetry.update();
                    break;
                }
            } else {
                telemetry.addData("Team Prop Location AXIS ", "LEFT AXIS");
                telemetry.update();
                spikeDetected = "LEFT";
               // leftFunction();
                telemetry.update();
                break;
            }
        }

      timer.reset();
        } else{
            telemetry.addData("No Object detected yet", "Trying Again");
            if(timer.seconds()>=5) {
                telemetry.addData("Team Prop Location AXIS ", "LEFT AXIS");
                telemetry.update();
                spikeDetected = "LEFT";
               // leftFunction();
            }
            telemetry.update();
        }

        //set position

    }   // end class

    private void noPropFound() {
        //Turn Left  and face  towards center stage board
        //Move to April tag-id=1
        //Drop Pixel
        //Park
        robot.Drive.MoveRobotToPosition(0.3, 3);
        sleep(700);
        robot.Drive.MoveRobotToPositionStrafe(-0.3, -40);
        sleep(700);
        robot.Drive.MoveRobotToPosition(0.3, 15);
        sleep(700);

        robot.clawLow.setClawPosition(0.5);
        sleep(700);
        robot.clawHigh.setClawPosition(0.5);
        sleep(700);

        robot.Drive.MoveRobotToPosition(-0.3, -16);
        sleep(700);
        // Send telemetry message to signify robot waiting;
        robot.Drive.MoveRobotToPositionStrafe(-0.3, -10);
        sleep(1500);
    }

    private void leftFunction() {
        //Strafe to location
        robot.Drive.MoveRobotToPositionStrafe(0.5, 4);
        sleep(700);


        robot.Drive.MoveRobotToPosition(0.5, 19);
        sleep(700);

        robot.Drive.pointTurn(-135, 0.5);
       sleep(700);

        robot.Drive.MoveRobotToPosition(0.3, 12);
        sleep(700);

        //move robot to TeamProp
      //  robot.Drive.MoveRobotToPosition(0.3, 24);
       // sleep(700);

        //Drop Pixel
        robot.clawLow.setClawPosition(0.5);
        sleep(700);

        //move back
        robot.Drive.MoveRobotToPosition(-0.5, -19);
        sleep(700);

        //Trun towards Center Stage
        robot.Drive.pointTurn(305, 0.3);
        sleep(700);

        //Move towards Center Stage
        robot.Drive.MoveRobotToPosition(0.3, 26);
        sleep(700);

        //Lift Arm
        robot.arm.SetMotorPower(0.75);
        sleep(700);

        //Drop Pixel
        robot.clawHigh.setClawPosition(0.5);
        sleep(700);
        //Drop Arm
        robot.arm.SetMotorPower(-0.75);
        sleep(700);

        robot.Drive.MoveRobotToPosition(-0.5, -2);
        sleep(700);

        //Strafe to Parking
        robot.Drive.MoveRobotToPositionStrafe(.5, 22);
        sleep(700);

        //Drive to Parking
        robot.Drive.MoveRobotToPosition(0.5, 12);
        sleep(15000);
    }

    private void centerFunction() {

        robot.Drive.MoveRobotToPositionStrafe(0.5, 2);
        sleep(700);

        //move robot to TeamProp
        robot.Drive.MoveRobotToPosition(0.5, 30);
        sleep(700);


        //Drop Pixel
        robot.clawLow.setClawPosition(0.5);
        sleep(700);

        //move robot to TeamProp
        robot.Drive.MoveRobotToPosition(-0.5, -6);
        sleep(700);

        //Trun towards Center Stage
        robot.Drive.pointTurn(170, 0.5);
        sleep(700);

        //Move towards Center Stage
        robot.Drive.MoveRobotToPosition(0.5, 30);
        sleep(700);

        //Strafe to Parking
        robot.Drive.MoveRobotToPositionStrafe(-0.5, -2);
        sleep(700);

        //Lift Arm
        robot.arm.SetMotorPower(0.75);
         //robot.arm.driveLiftToPosition(.3,8);
        sleep(1500);
        //Drop Pixel
        robot.clawHigh.setClawPosition(0.5);
        sleep(700);

        //Drop Arm
         robot.arm.SetMotorPower(-0.75);
        sleep(700);

        //Strafe to Parking
        robot.Drive.MoveRobotToPositionStrafe(0.5, 26);
        sleep(700);

        //Drive to Parking
        robot.Drive.MoveRobotToPosition(0.5, 12);
        sleep(15000);
    }

    private void rightFunction() {
        //move robot to TeamProp
        robot.Drive.MoveRobotToPosition(0.5, 19);
        sleep(700);

        //Strafe to center
        robot.Drive.MoveRobotToPositionStrafe(0.5, 8);
        sleep(700);


        //Drop Pixel
        robot.clawLow.setClawPosition(0.5);
        sleep(700);

        robot.Drive.MoveRobotToPosition(-0.5, -8);
        sleep(700);

        //Trun towards Center Stage
        robot.Drive.pointTurn(170, 0.5);
        sleep(700);

        //Move towards Center Stage
        robot.Drive.MoveRobotToPosition(0.5, 28);
        sleep(700);

        robot.Drive.MoveRobotToPositionStrafe(-0.5, -16);
        sleep(700);

        //Lift Arm
        robot.arm.SetMotorPower(0.75);
       // robot.arm.driveLiftToPosition(.3,8);
        sleep(1500);
        //Drop Pixel
        robot.clawHigh.setClawPosition(0.5);
        sleep(700);

        //Drop Arm
        robot.arm.SetMotorPower(-0.75);
        sleep(700);

        //Strafe to Parking
        robot.Drive.MoveRobotToPositionStrafe(0.5, 24);
        sleep(700);

        //Drive to Parking
        robot.Drive.MoveRobotToPosition(0.5, 8);
        sleep(15000);
    }


}
