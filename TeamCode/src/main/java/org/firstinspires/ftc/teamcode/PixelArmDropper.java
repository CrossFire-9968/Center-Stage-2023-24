package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class PixelArmDropper {
    private DcMotor ArmMotor;

    public void init() {
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        ArmMotor.setPower(0.0);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void drive() {
        telemetry.addData("Arm Position:", ArmMotor.getCurrentPosition());
    }
}
