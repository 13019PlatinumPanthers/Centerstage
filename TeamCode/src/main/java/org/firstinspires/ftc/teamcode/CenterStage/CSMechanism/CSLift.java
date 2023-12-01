package org.firstinspires.ftc.teamcode.CenterStage.CSMechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CSLift {
    Telemetry telemetry;
    public DcMotor liftMotor1;
    public DcMotor liftMotor2;


    public CSLift(DcMotor armleftMotor,DcMotor armRightMotor, Telemetry telemetry ) {
        this.telemetry = telemetry;
       // setup( hardwareMap );
        this.liftMotor1 = armleftMotor;
        this.liftMotor2 = armRightMotor;
        setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.ZeroPowerBehavior.BRAKE);

    }


    public void SetMotorPower(double Power){
        liftMotor1.setPower(Power);
        liftMotor2.setPower(-Power);
    }

    public void driveLiftToPosition(double power, int armPosition) {
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor1.setTargetPosition(-armPosition);
        liftMotor2.setTargetPosition(armPosition);

        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor1.setPower(power);
        liftMotor2.setPower(power);

        while(liftMotor1.isBusy()&&liftMotor2.isBusy()){
        }

    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior l1Behavior, DcMotor.ZeroPowerBehavior l2Behavior){
        liftMotor1.setZeroPowerBehavior( l1Behavior );
        liftMotor2.setZeroPowerBehavior( l2Behavior );

    }
}


