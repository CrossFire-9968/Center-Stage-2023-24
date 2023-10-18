package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Robot TeleOp")
public class RobotControl extends OpMode {
    MecanumDrive mecDrive = new MecanumDrive();


    @Override
    public void init() {
        mecDrive.init();
    }

    @Override
    public void loop() {
        mecDrive.drive();
        telemetry.update();
    }
}