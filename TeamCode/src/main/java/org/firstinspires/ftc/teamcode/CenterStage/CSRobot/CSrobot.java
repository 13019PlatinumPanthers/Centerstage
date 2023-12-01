package org.firstinspires.ftc.teamcode.CenterStage.CSRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStage.CSMechanism.CSDriveTrain;
import org.firstinspires.ftc.teamcode.CenterStage.CSMechanism.CSIntake;
import org.firstinspires.ftc.teamcode.CenterStage.CSMechanism.CSIntakeClawFlight;
import org.firstinspires.ftc.teamcode.CenterStage.CSMechanism.CSIntakeClawHigh;
import org.firstinspires.ftc.teamcode.CenterStage.CSMechanism.CSIntakeClawLow;
import org.firstinspires.ftc.teamcode.CenterStage.CSMechanism.CSLift;
import org.firstinspires.ftc.teamcode.CenterStage.CSSensor.CSDistanceSensor;
import org.firstinspires.ftc.teamcode.OldReferencesPowerPlay.Robot.Sensors.Limit_Switch;
import org.firstinspires.ftc.teamcode.OldReferencesPowerPlay.Robot.Sensors.distance_sensor;

public class CSrobot {
    public OpMode opMode;
    public HardwareMap hardwareMap;
    Telemetry telemetry;

    //public distance_sensor frontDist;
    public CSLift arm;
    public CSIntakeClawHigh clawHigh;
    public CSIntakeClawLow clawLow;

    public CSIntakeClawFlight clawFlight;

    public CSIntake intakeMechanism;

    public CSDriveTrain Drive;
    // public distance_sensor left;
    //public distance_sensor right;

    //Distance sendor is used to calculate the distance between pixel and alert the driver pixel is ready
    public CSDistanceSensor senseDistance;
    public Limit_Switch limit1;

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private DcMotor armleftMotor    = null;  //  Used to control the left back drive wheel
    private DcMotor armRightMotor   = null;  //  Used to control the right back drive wheel

    private DcMotor intakeMotor   = null;  //  Used to control the right back drive wheel



    public CSrobot(OpMode op ) {

        opMode = op;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        //Distance sensor : you can use this as a regular DistanceSensor.
        //sensorDistance = hardwareMap.get(DistanceSensor.class, "distFront");


        //Drive Train
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        Drive = new CSDriveTrain( hardwareMap,leftFrontDrive,rightFrontDrive,leftBackDrive, rightBackDrive, opMode.telemetry);

        //Claw
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        clawLow = new CSIntakeClawLow(hardwareMap,"clawLow",opMode.telemetry);
        clawHigh = new CSIntakeClawHigh(hardwareMap,"clawHigh",opMode.telemetry);
        clawFlight = new CSIntakeClawFlight(hardwareMap,"clawFlight",opMode.telemetry);

        // Arm
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        armleftMotor  = hardwareMap.get(DcMotor.class, "armLeft");
        armRightMotor = hardwareMap.get(DcMotor.class, "armRight");
        arm = new CSLift(armleftMotor,armRightMotor, opMode.telemetry);


        // Arm
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        intakeMotor  = hardwareMap.get(DcMotor.class, "intake");
        intakeMechanism = new CSIntake(intakeMotor, opMode.telemetry);

        //Distance Sensors
        senseDistance = new CSDistanceSensor(hardwareMap, "DistanceBack", opMode.telemetry);

        //


    }
}