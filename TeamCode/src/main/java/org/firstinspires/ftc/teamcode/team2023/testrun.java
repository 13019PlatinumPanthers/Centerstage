package org.firstinspires.ftc.teamcode.team2023;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Disabled
@Autonomous
public class testrun extends OpMode {

   // public DcMotor frontLeftMotor = null;
    //public DcMotor backLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backRightMotor = null;

    private final ElapsedTime runtime = new ElapsedTime();

    AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .build();


   /* VisionPortal visionPortal = new VisionPortal.Builder()
            .addProcessor(aprilTagProcessor)
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .setCameraResolution(new Size(640, 480))
            .build();
            */





    public void runOpMode() throws InterruptedException {

    }


    @Override
    public void init() {
      //  frontLeftMotor = hardwareMap.dcMotor.get("FrontLeft");
      //  backLeftMotor = hardwareMap.dcMotor.get("BackLeft");
        frontRightMotor = hardwareMap.dcMotor.get("FrontRight");
        backRightMotor = hardwareMap.dcMotor.get("BackRight");



      //  frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void loop() {
        //frontRightMotor.setPower(1);
        forward(-1);

        telemetry.addLine("testing 111");
       /* if(aprilTagProcessor.getDetections().size()>0){
            AprilTagDetection tagDetection = aprilTagProcessor.getDetections().get(0);
            telemetry.addLine("testing");

        }*/

    }

    void forward(int power){
       // frontLeftMotor.setPower(power);
      //  backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
        if(runtime.seconds()<2){
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);
        }
    }
    void backwards(int power){
       // frontLeftMotor.setPower(-power);
      //  backLeftMotor.setPower(-power);
        frontRightMotor.setPower(-1);
        backRightMotor.setPower(-power);
    }
}
