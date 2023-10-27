package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PixelArmDropper {
    public DcMotor ArmMotor;

    public void init(HardwareMap hwMap) {
        ArmMotor = hwMap.get(DcMotor.class, "ArmMotor");
        ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        ArmMotor.setPower(0.0);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void drive(Gamepad gamepad, Telemetry telemetry) {
        double armSpeedUp = 0.2;
        double armSpeedDown = 0.1;
        if (gamepad.a) {
            ArmMotor.setPower(armSpeedDown);
            ArmMotor.setTargetPosition(0);
            ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addLine("Moving Down");
        }

        if (gamepad.y) {
            ArmMotor.setPower(armSpeedUp);
            ArmMotor.setTargetPosition(750);
            ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addLine("Moving Up");
         }

        telemetry.addData("Arm Position:", ArmMotor.getCurrentPosition());
    }
}
