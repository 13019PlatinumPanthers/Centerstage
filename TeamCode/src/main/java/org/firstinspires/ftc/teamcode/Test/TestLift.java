package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OldReferencesPowerPlay.Robot.GBrobot;
@Autonomous
@Disabled
public class TestLift extends LinearOpMode {
    public GBrobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new GBrobot(this);
        waitForStart();

        robot.claw.setClawPosition(0.0);
        sleep(1000);
        robot.lift.driveLiftToPosition(0.5, 7);
        sleep(1000);
        robot.claw.setClawPosition(0.25);
        sleep(300);
    }
}
