package org.firstinspires.ftc.teamcode.CenterStage.CSSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CSLimit_Switch {
    Telemetry telemetry;
    TouchSensor limit1;

    public CSLimit_Switch(HardwareMap hardwareMap, String limitSwitchName, Telemetry telemetry ) {
        this.telemetry = telemetry;
        setup( hardwareMap, limitSwitchName);
    }

    public void setup( HardwareMap hardwareMap, String limitSwitchName ) {
        limit1 = hardwareMap.get(TouchSensor.class, limitSwitchName );
    }

   public boolean isPressed(){
        return limit1.isPressed();
   }
}
