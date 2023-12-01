package org.firstinspires.ftc.teamcode.CenterStage.unused;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStage.CSMechanism.CSDriveTrain;
import org.firstinspires.ftc.teamcode.CenterStage.CSMechanism.CSIntake;
import org.firstinspires.ftc.teamcode.CenterStage.CSMechanism.CSIntakeClawLow;
import org.firstinspires.ftc.teamcode.OldReferencesPowerPlay.Robot.Sensors.Limit_Switch;

public class OccraCSrobot {
    public OpMode opMode;
    public HardwareMap hardwareMap;
    Telemetry telemetry;

    //public distance_sensor frontDist;
      public CSIntakeClawLow clawLow;

    public CSIntake intakeMechanism;

    public CSDriveTrain Drive;
    // public distance_sensor left;
    //public distance_sensor right;
    public Limit_Switch limit1;

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private final DcMotor armleftMotor    = null;  //  Used to control the left back drive wheel
    private final DcMotor armRightMotor   = null;  //  Used to control the right back drive wheel

    private DcMotor intakeMotor   = null;  //  Used to control the right back drive wheel

    public OccraCSrobot(OpMode op ) {

        opMode = op;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;


        //Drive Train
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftfront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftback");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightback");
        Drive = new CSDriveTrain(hardwareMap,leftFrontDrive,rightFrontDrive,leftBackDrive, rightBackDrive, opMode.telemetry);

        //Claw
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        clawLow = new CSIntakeClawLow(hardwareMap,"ServoClawLow",opMode.telemetry);


        // Arm
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        intakeMotor  = hardwareMap.get(DcMotor.class, "intake");
        intakeMechanism = new CSIntake(armleftMotor, opMode.telemetry);


    }
}