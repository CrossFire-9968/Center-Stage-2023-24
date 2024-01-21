package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Blinkin {
    public RevBlinkinLedDriver blinkin;

    public void init(HardwareMap hwMap){
        blinkin = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public void setColor(RevBlinkinLedDriver.BlinkinPattern color){
        blinkin.setPattern(color);
    }
}
