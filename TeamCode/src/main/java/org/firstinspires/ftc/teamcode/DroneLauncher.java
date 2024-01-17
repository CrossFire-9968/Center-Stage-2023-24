package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncher {

    public Servo Launcher;
    double launcherMin = 0.4;
    double launcherMax = 1.0;

    public void init(HardwareMap hwMap) {
        Launcher = hwMap.get(Servo.class, "Launcher");
        Launcher.setDirection(Servo.Direction.REVERSE);
        Launcher.setPosition(launcherMin);
    }

    /**
     * <p> Method operate the launch mechanism used during teleop to release the drone </p>
     */
    public void control(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            Launcher.setPosition(launcherMax);
        }
    }
}

