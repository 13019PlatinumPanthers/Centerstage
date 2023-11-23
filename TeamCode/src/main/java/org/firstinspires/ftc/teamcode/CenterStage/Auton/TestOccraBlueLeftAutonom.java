/* Copyright (c) 2023 FIRST. All rights reserved.
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

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * For an introduction to AprilTags, see the ftc-docs link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@Autonomous
@Disabled
public class TestOccraBlueLeftAutonom extends LinearOpMode {
    private static  String teamPropLocation = "NONE";
    private static  boolean detectAprilTag = false;
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
    public CSrobot robot;
    String park;
    //Change this by keeping the robot facing the spikes
    //Left Spike
    //X-Axis
    double leftXLowerBound = 120.0;
    double leftXUpperBound = 120.0;
    //Y-Axis
    double leftYLowerBound = 120.0;
    double leftYUpperBound = 120.0;
    //Center Spike
    //X-Axis
    double centerXLowerBound = 120.0;
    double centerXUpperBound = 120.0;
    //X-Axis
    double centerYLowerBound = 120.0;
    double centerYUpperBound = 120.0;

   // public static String teamPropLocation = "NONE";

    public static final double MID_SERVO = 0.5;
    public static final double CLAW_SPEED = 0.02;
    double clawOffset = 0;

    //my custom model asset
    private static final String TFOD_MODEL_ASSET = "blueAsset.tflite";

    @Override
    public void runOpMode() {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)


        robot = new CSrobot(this);
        waitForStart();
        // Wait for the game to start (driver presses PLAY)

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "initialize");    //

        //Initaiate Tensor flow
        //Use LinearOpMode linearOpMode,double leftXLowerBound,double leftXUpperBound,double leftYLowerBound,double leftYUpperBound,double centerXLowerBound,double centerXUpperBound,double centerYLowerBound,double centerYUpperBound
        teamPropLocation = new TensorFlowObjectDetection().findPropLocation(this, TFOD_MODEL_ASSET, leftXLowerBound, leftXUpperBound, leftYLowerBound, leftYUpperBound, centerXLowerBound, centerXUpperBound, centerYLowerBound, centerYUpperBound);
        if (teamPropLocation.equals("LEFT")) {

            //move forward + turn left +  drop pixel
            robot.Drive.MoveRobotToPosition(.3, 25);
            robot.Drive.pointTurn(-95, 0.3);
            robot.clawLow.setClawPosition(0.25);

            //Turn Left  and face  towards center stage board
            //Move Towards April Tag
            detectAprilTag = new DriveToAprilTag().findAprilTagLocation(1, this, robot.Drive.FrontLeft, robot.Drive.FrontRight, robot.Drive.BackLeft, robot.Drive.BackRight);

            //Drop Pixel
            robot.arm.driveLiftToPosition(1, 100);
            robot.clawHigh.setClawPosition(0.25);

            //Park
            //parkLeft();

        } else if (teamPropLocation.equals("CENTER")) {
            //Turn Left  and face  towards center stage board

            //Move to April tag-id=2
            //Drop Pixel
            //Park
        } else if (teamPropLocation.equals("RIGHT")) {
            //Turn completely by 180Degree and  face towards center stage board
            //Move to April tag-id=3
            //Drop Pixel
            //Park
        } else if (teamPropLocation.equals("None")) {
            //Turn Left  and face  towards center stage board
            //Move to April tag-id=1
            //Drop Pixel
            //Park
        }

        robot.Drive.MoveRobotToPosition(0.3, 4);
        sleep(700);
        robot.Drive.MoveRobotToPositionStrafe(0.3, 40);
        sleep(700);
        robot.Drive.MoveRobotToPosition(0.3, 15);
        sleep(700);


        robot.clawLow.setClawPosition(0.5);
        sleep(700);


        robot.Drive.MoveRobotToPosition(-0.3, -16);
        sleep(700);
        // Send telemetry message to signify robot waiting;
        robot.Drive.MoveRobotToPositionStrafe(0.3, 10);
        sleep(700);


        telemetry.addData(">", "setpower");
        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "drive again");    //
        telemetry.update();
        sleep(700);


    }


}