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

package org.firstinspires.ftc.teamcode.CenterStage.DriverControlMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CenterStage.Auton.CSrobot;
import org.firstinspires.ftc.teamcode.Robot.GBrobot;

/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp
public class CSRegularDriverMode_Linear extends OpMode {


        public CSrobot robot;

        @Override
        public void init() {
        this.robot = new CSrobot(this);
        robot.Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.ZeroPowerBehavior.BRAKE);
    }

        @Override
        public void loop() {
        double slowFactor = 1;


        //gamepad 1
        // Drivetrain associated 1 Drivetrain and Intake
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x * 1.1;
        double turn = -gamepad1.right_stick_x;
        if (gamepad1.right_bumper) {
                slowFactor = .5;
            }


        double denominator = Math.max(Math.abs(drive)+Math.abs(strafe)+Math.abs(turn),1);

        double fl = ((drive+strafe+turn)/denominator)*slowFactor;
        double bl = ((drive-strafe+turn)/denominator)*slowFactor;
        double fr = ((drive-strafe-turn)/denominator)*slowFactor;
        double br = ((drive+strafe-turn)/denominator)*slowFactor;
        robot.Drive.setMotorPower(fl, bl, fr, br);




        //gamepad 2 is for Arm and Claw
        // Arm lift/down/Stop
        double lift = -gamepad2.left_stick_y*0.8;
        ///lift =Range.clip(lift,-1.0,1.0);
        robot.arm.SetMotorPower(lift);

       // double drop = gamepad2.left_stick_y*0.8;
       // robot.arm.SetMotorPower(-drop);

        if (gamepad2.b) {

            //The motor stops and then floats: an external force attempting to turn the motor is not met with active resistence.
          //  robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.ZeroPowerBehavior.BRAKE);
            while(true){
                if(Math.abs(gamepad2.left_stick_y)>0.1)break;
                else
                    robot.arm.SetMotorPower(-0.3);
            }
            }



            //Intake Mechanism
            if (gamepad2.x) {
                robot.intakeMechanism.SetMotorPower(1);
            }
            else if (gamepad2.y){
                robot.intakeMechanism.SetMotorPower(-1);
                robot.clawHigh.setClawPosition(1);

            }
            else if(gamepad2.a){
                robot.intakeMechanism.SetMotorPower(0.0);

            }

       // double cOpen = gamepad2.right_stick_x*0.8;
       // robot.clawHigh.SetMotorPower(cOpen);
       // double cClose = gamepad2.right_stick_y*0.8;
      //  robot.clawHigh.SetMotorPower(-cClose);



            // Claw controls
            if(gamepad2.left_bumper){
                robot.clawHigh.setClawPosition(1);
            }

            if(gamepad2.right_bumper){
                robot.clawHigh.setClawPosition(0);
            }


//flight control
            if(gamepad2.dpad_up){
                robot.clawFlight.setClawPosition(1);
            }
            if(gamepad2.dpad_down){
                robot.clawFlight.setClawPosition(-1);
            }
            if(gamepad2.dpad_left){
                robot.clawFlight.setClawPosition(0);
            }
 if(gamepad2.dpad_right){
            robot.clawFlight.setClawPosition(0);
}


      /*  if(gamepad2.a){
            robot.lift.driveLiftToPosition(0.5, 500);
        }
        if (gamepad2.b){
            robot.lift.driveLiftToPosition(0.5, 1600);
        }
        if (gamepad2.y){
            robot.lift.driveLiftToPosition(0.5, 2800);
        }
        if (gamepad2.x){
            robot.lift.driveLiftToPosition(0.5, 3900);
        }
        if (gamepad2.back){
            if(robot.limit1.isPressed()){
                telemetry.addData("IS PRESSED:" , " true");
                robot.lift.SetMotorPower(0);
            }
            else{
                telemetry.addData("IS PRESSED", "false");
                robot.lift.SetMotorPower(-1);
            }
            telemetry.update();
        }
*/
    }
    }

