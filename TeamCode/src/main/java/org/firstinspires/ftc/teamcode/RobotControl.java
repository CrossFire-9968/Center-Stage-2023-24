package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Robot TeleOp")
public class RobotControl extends OpMode {
    private DcMotor MotorLRear;
    private DcMotor MotorRRear;
    private DcMotor MotorLFront;
    private DcMotor MotorRFront;
    double driveSpeed;
    double turnSpeed;
    double strafeSpeed;
    boolean wasX = true;
    boolean wasY = false;
    double driveSensitivity;

    @Override
    public void init( ) {
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

    @Override
    public void loop() {
        if (gamepad1.x || wasX) {
            drive();
            wasX = true;
            wasY = false;
            telemetry.addLine("Drive Normal");
        }
        if (gamepad1.y || wasY) {
            adjustedDrive();
            wasY = true;
            wasX = false;
            telemetry.addLine("Drive Adjusted");
        }


        telemetry.addData("speed Forward", driveSpeed);
        telemetry.update();
    }
    public void drive() {
        driveSensitivity = 0.7;
        strafeSpeed = gamepad1.left_stick_x * driveSensitivity;
        turnSpeed = gamepad1.right_stick_x * driveSensitivity;
        driveSpeed = -gamepad1.left_stick_y * driveSensitivity;

        double RFrontPower = driveSpeed - turnSpeed - strafeSpeed;
        double LFrontPower = driveSpeed + turnSpeed + strafeSpeed;
        double RRearPower = driveSpeed - turnSpeed + strafeSpeed;
        double LRearPower = driveSpeed + turnSpeed - strafeSpeed;

        double maxPower = driveSensitivity;

        if (RFrontPower > maxPower){
            maxPower = RFrontPower;
        }
        if (LFrontPower > maxPower){
            maxPower = LFrontPower;
        }
        if (RRearPower > maxPower){
            maxPower = RRearPower;
        }
        if (LRearPower > maxPower){
            maxPower = LRearPower;
        }

        RFrontPower = (RFrontPower / maxPower);
        LFrontPower = (LFrontPower / maxPower);
        RRearPower = (RRearPower / maxPower);
        LRearPower = (LRearPower / maxPower);

        MotorRFront.setPower(RFrontPower);
        MotorLFront.setPower(LFrontPower);
        MotorRRear.setPower(RRearPower);
        MotorLRear.setPower(LRearPower);

        telemetry.addData("MaxPower:", maxPower);
    }

    public void adjustedDrive() {
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