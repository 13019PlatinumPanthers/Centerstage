package org.firstinspires.ftc.teamcode.CenterStage.Auton;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CSIntake {
    Telemetry telemetry;
    public DcMotor intakeMotor1;
   // public DcMotor liftMotor2;


    public CSIntake(DcMotor armleftMotor, Telemetry telemetry ) {
        this.telemetry = telemetry;
       // setup( hardwareMap );
        this.intakeMotor1 = armleftMotor;
       // this.liftMotor2 = armRightMotor;
    }


    public void SetMotorPower(double Power){
        intakeMotor1.setPower(Power);
        // liftMotor2.setPower(-Power);
    }

    public void driveIntakeToPosition(double power, int armPosition) {
        intakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeMotor1.setTargetPosition(-armPosition);
       // liftMotor2.setTargetPosition(armPosition);

        intakeMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       // liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeMotor1.setPower(power);
       // liftMotor2.setPower(power);

        while(intakeMotor1.isBusy()){
        }

    }
}
