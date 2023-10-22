package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ArmControl {
    private DcMotor ArmMotor;

    public void init() {
        ArmMotor = hardwareMap.get(DcMotor.class, "MotorRFront");
        ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //ArmMotor.setMotorType(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setPower(0.0);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drive() {
        telemetry.addData("Arm Position:", ArmMotor.getCurrentPosition());
    }
}
