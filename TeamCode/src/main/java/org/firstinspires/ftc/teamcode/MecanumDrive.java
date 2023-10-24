package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDrive {
    public DcMotor motor_LR;
    public DcMotor motor_RR;
    public DcMotor motor_LF;
    public DcMotor motor_RF;
    double LFrontPower;
    double RFrontPower;
    double RRearPower;
    double LRearPower;
    final double driveSensitivity = 0.7;


    public void init(HardwareMap hwMap) {
        motor_LF = hwMap.get(DcMotor.class, "Motor_LF");
        motor_LF.setDirection(DcMotorSimple.Direction.FORWARD);

        motor_RF = hwMap.get(DcMotor.class, "Motor_RF");
        motor_RF.setDirection(DcMotorSimple.Direction.REVERSE);

        motor_RR = hwMap.get(DcMotor.class, "Motor_RR");
        motor_RR.setDirection(DcMotorSimple.Direction.FORWARD);

        motor_LR = hwMap.get(DcMotor.class, "Motor_LR");
        motor_LR.setDirection(DcMotorSimple.Direction.REVERSE);

        setMecanumPowers(0.0);
    }


    public void manualDrive(Gamepad gamepad, Telemetry telemetry) {
        double strafeSpeed = gamepad.left_stick_x;
        double turnSpeed = gamepad.right_stick_x;
        double driveSpeed = gamepad.left_stick_y;

        // Raw drive power for each motor from joystick inputs
        LFrontPower = driveSpeed - turnSpeed - strafeSpeed;
        RFrontPower = driveSpeed + turnSpeed + strafeSpeed;
        RRearPower = driveSpeed + turnSpeed - strafeSpeed;
        LRearPower = driveSpeed - turnSpeed + strafeSpeed;

//        // Cubing power values to give finer control at slow speeds
        LFrontPower = Math.pow(LFrontPower, 3);
        RFrontPower = Math.pow(RFrontPower, 3);
        RRearPower = Math.pow(RRearPower, 3);
        LRearPower = Math.pow(LRearPower, 3);

        // Find max drive power
        double max = 1.0;
        max = Math.max(max, Math.abs(LFrontPower));
        max = Math.max(max, Math.abs(RFrontPower));
        max = Math.max(max, Math.abs(RRearPower));
        max = Math.max(max, Math.abs(LRearPower));

        // Ratio drive powers
        LFrontPower = (LFrontPower / max);
        RFrontPower = (RFrontPower / max);
        RRearPower =  (RRearPower / max);
        LRearPower =  (LRearPower / max);

        telemetry.addData("Max: ", max);

        // Set motor speed
        setMecanumPowers(LFrontPower, RFrontPower, RRearPower, LRearPower);
    }

    // Set all mecanum powers
    protected void setMecanumPowers(double power) {
        motor_LF.setPower(power);
        motor_RF.setPower(power);
        motor_RR.setPower(power);
        motor_LR.setPower(power);
    }


    protected void setMecanumPowers(double LFpower, double RFpower, double RRpower, double LRpower) {
        motor_LF.setPower(driveSensitivity * LFpower);
        motor_RF.setPower(driveSensitivity * RFpower);
        motor_RR.setPower(driveSensitivity * RRpower);
        motor_LR.setPower(driveSensitivity * LRpower);
    }
}