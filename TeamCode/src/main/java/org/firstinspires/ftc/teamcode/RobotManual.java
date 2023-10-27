package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Robot Manual")
public class RobotManual extends OpMode {
    MecanumDrive mecDrive = new MecanumDrive();
    PixelArmDropper pixelArm = new PixelArmDropper();

    @Override
    public void init() {
        mecDrive.init(hardwareMap);
        pixelArm.init(hardwareMap);
        
        telemetry.addLine("Begin initializations");
        telemetry.addLine("... Mecanum init complete");
        telemetry.addLine("End initializations");
    }

    @Override
    public void loop() {
        mecDrive.manualDrive(gamepad1, telemetry);
        pixelArm.drive(gamepad1, telemetry);
        telemetry.addData("Arm position: ", pixelArm.ArmMotor.getCurrentPosition());
        telemetry.update();
    }
}