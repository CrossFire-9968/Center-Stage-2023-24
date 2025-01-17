package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDrive {
    public DcMotor motor_LR;
    public DcMotor motor_RR;
    public DcMotor motor_LF;
    public DcMotor motor_RF;
    final double driveSensitivity = 0.7;
    double LFrontPower;
    double RFrontPower;
    double RRearPower;
    double LRearPower;
    double strafeMax = 1.0;
    boolean strafeByJoystick = false;
    boolean backButtonHeld = false;

    public void init(HardwareMap hwMap){
        motor_LF = hwMap.get(DcMotor.class, "Motor_LF");
        motor_LF.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_RF = hwMap.get(DcMotor.class, "Motor_RF");
        motor_RF.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motor_RR = hwMap.get(DcMotor.class, "Motor_RR");
        motor_RR.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motor_LR = hwMap.get(DcMotor.class, "Motor_LR");
        motor_LR.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setAllMecanumPowers(0.0);
    }

    public void manualDrive(Gamepad gamepad, Telemetry telemetry) {
        double turnSpeed = gamepad.right_stick_x;
        double driveSpeed = gamepad.left_stick_y;
        double strafeSpeed = 0.0;

        if (gamepad.back && !backButtonHeld)
        {
            strafeByJoystick = !strafeByJoystick;
            backButtonHeld = true;
        }

        if (!gamepad.back) {
            backButtonHeld = false;
        }

        if (strafeByJoystick) {
            strafeSpeed = gamepad.left_stick_x;
            telemetry.addLine("Strafe by Joystick");
        }
        else {
            if (gamepad.left_bumper) {
                strafeSpeed = -strafeMax;
            }
            else if (gamepad.right_bumper) {
                strafeSpeed = strafeMax;
            }
            telemetry.addLine("Strafe by Buttons");
        }

        // Raw drive power for each motor from joystick inputs
        LFrontPower = driveSpeed - turnSpeed - strafeSpeed;
        RFrontPower = driveSpeed + turnSpeed + strafeSpeed;
        RRearPower = driveSpeed + turnSpeed - strafeSpeed;
        LRearPower = driveSpeed - turnSpeed + strafeSpeed;

        // Find which motor power command is the greatest. If not motor
        // is greater than 1.0 (the max motor power possible) just set it by default
        // to 1.0 so the ratiometric calculation we do next does not
        // inadvertently increase motor powers.
        double max = 1.0;
        max = Math.max(max, Math.abs(LFrontPower));
        max = Math.max(max, Math.abs(RFrontPower));
        max = Math.max(max, Math.abs(RRearPower));
        max = Math.max(max, Math.abs(LRearPower));

        // Ratiometric calculation that proportionally reduces all powers in cases where on
        // motor input is greater than 1.0. This keeps the driving feel consistent to the driver.
        LFrontPower = (LFrontPower / max);
        RFrontPower = (RFrontPower / max);
        RRearPower = (RRearPower / max);
        LRearPower = (LRearPower / max);

        // Set motor speed
        setEachMecanumPower(LFrontPower, RFrontPower, RRearPower, LRearPower);
    }

    // Set all mecanum powers
    protected void setAllMecanumPowers(double power) {
        motor_LF.setPower(power);
        motor_RF.setPower(power);
        motor_RR.setPower(power);
        motor_LR.setPower(power);
    }

    protected void setEachMecanumPower(double LFpower, double RFpower, double RRpower, double LRpower) {
        motor_LF.setPower(driveSensitivity * LFpower * 1.03);
        motor_RF.setPower(driveSensitivity * RFpower);
        motor_RR.setPower(driveSensitivity * RRpower);
        motor_LR.setPower(driveSensitivity * LRpower);
    }
}
