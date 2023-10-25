package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot TeleOp")
public class RobotControl extends OpMode {
    MecanumDrive mecDrive = new MecanumDrive();
    PixelArmDropper pixelDrop = new PixelArmDropper();

    @Override
    public void init() {
        mecDrive.init();
        pixelDrop.init();
    }

    @Override
    public void loop() {
        mecDrive.drive();
        pixelDrop.drive();
        telemetry.update();
    }
}