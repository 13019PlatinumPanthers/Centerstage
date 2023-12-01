package org.firstinspires.ftc.teamcode.team2023;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp
public class itakeMotorTest extends LinearOpMode {
   // GBrobot robot;
    public DcMotor intakeMotor = null;

    @Override
    public void runOpMode() {

        // Define and initialize intake.
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        //robot = new GBrobot( this );
        telemetry.addLine( "Ready!" );
        telemetry.update( );
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a){
                intakeMotor.setPower(1);
            }
            if(gamepad1.x){
                intakeMotor.setPower(-1);
            }
            if(gamepad1.b){
                intakeMotor.setPower(0);
            }

        }


    }


}
