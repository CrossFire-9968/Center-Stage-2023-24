package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MecanumDrive {
    private DcMotor MotorLRear;
    private DcMotor MotorRRear;
    private DcMotor MotorLFront;
    private DcMotor MotorRFront;
    double driveSpeed;
    double turnSpeed;
    double strafeSpeed;
    double driveSensitivity;
    public void init() {
        MotorLRear = hardwareMap.get(DcMotor.class, "MotorLRear");
        MotorLRear.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorLRear.setPower(0.0);

        MotorRRear = hardwareMap.get(DcMotor.class, "MotorRRear");
        MotorRRear.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorRRear.setPower(0.0);

        MotorLFront = hardwareMap.get(DcMotor.class, "MotorLFront");
        MotorLFront.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorLFront.setPower(0.0);

        MotorRFront = hardwareMap.get(DcMotor.class, "MotorRFront");
        MotorRFront.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorRFront.setPower(0.0);
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

        MotorRFront.setPower(RFrontPower);
        MotorLFront.setPower(LFrontPower);
        MotorRRear.setPower(RRearPower);
        MotorLRear.setPower(LRearPower);

        telemetry.addData("MaxPowerSq:", maxPowerSq);
    }
}
