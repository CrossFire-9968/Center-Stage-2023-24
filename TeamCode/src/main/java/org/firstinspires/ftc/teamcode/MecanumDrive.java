package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MecanumDrive {
    private DcMotor motor_LR = null;
    private DcMotor motor_RR = null;
    private DcMotor motor_LF = null;
    private DcMotor motor_RF = null;
    double driveSpeed;
    double turnSpeed;
    double strafeSpeed;
    double driveSensitivity;

    public void init() {
        motor_LR = hardwareMap.get(DcMotor.class, "MotorLR");
        motor_LR.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_LR.setPower(0.0);

        motor_RR = hardwareMap.get(DcMotor.class, "MotorRRear");
        motor_RR.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_RR.setPower(0.0);

        motor_LF = hardwareMap.get(DcMotor.class, "MotorLFront");
        motor_LF.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_LF.setPower(0.0);

        motor_RF = hardwareMap.get(DcMotor.class, "MotorRFront");
        motor_RF.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_RF.setPower(0.0);
    }

    public void drive() {
        driveSensitivity = 0.7;
        strafeSpeed = gamepad1.left_stick_x;
        turnSpeed = gamepad1.right_stick_x;
        driveSpeed = -gamepad1.left_stick_y;

        double RFrontPower = driveSpeed - turnSpeed - strafeSpeed;
        double LFrontPower = driveSpeed + turnSpeed + strafeSpeed;
        double RRearPower = driveSpeed - turnSpeed + strafeSpeed;
        double LRearPower = driveSpeed + turnSpeed - strafeSpeed;

        double RFrontPowerSq = Math.pow(RFrontPower, 3)  * driveSensitivity;
        double LFrontPowerSq = Math.pow(LFrontPower, 3)  * driveSensitivity;
        double RRearPowerSq = Math.pow(RRearPower, 3)  * driveSensitivity;
        double LRearPowerSq = Math.pow(LRearPower, 3)  * driveSensitivity;

        double maxPowerSq = driveSensitivity;

        if (RFrontPowerSq > maxPowerSq){
            maxPowerSq = RFrontPowerSq;
        }
        if (LFrontPowerSq > maxPowerSq){
            maxPowerSq = LFrontPowerSq;
        }
        if (RRearPowerSq > maxPowerSq){
            maxPowerSq = RRearPowerSq;
        }
        if (LRearPowerSq > maxPowerSq){
            maxPowerSq = LRearPowerSq;
        }

        RFrontPower = (RFrontPowerSq / maxPowerSq);
        LFrontPower = (LFrontPowerSq / maxPowerSq);
        RRearPower =  (RRearPowerSq / maxPowerSq);
        LRearPower =  (LRearPowerSq / maxPowerSq);

        motor_RF.setPower(RFrontPower);
        motor_LF.setPower(LFrontPower);
        motor_RR.setPower(RRearPower);
        motor_LR.setPower(LRearPower);

        telemetry.addData("MaxPowerSq:", maxPowerSq);
    }
}
