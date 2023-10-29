package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Robot Manual Flat")
public class RobotManual_flat extends OpMode {
    public DcMotor motor_LR;
    public DcMotor motor_RR;
    public DcMotor motor_LF;
    public DcMotor motor_RF;
    public DcMotor pixel_Motor;
    double LFrontPower;
    double RFrontPower;
    double RRearPower;
    double LRearPower;
    final double driveSensitivity = 0.7;


    @Override
    public void init() {
        motor_LF = hardwareMap.get(DcMotor.class, "Motor_LF");
        motor_LF.setDirection(DcMotorSimple.Direction.FORWARD);

        motor_RF = hardwareMap.get(DcMotor.class, "Motor_RF");
        motor_RF.setDirection(DcMotorSimple.Direction.REVERSE);

        motor_RR = hardwareMap.get(DcMotor.class, "Motor_RR");
        motor_RR.setDirection(DcMotorSimple.Direction.FORWARD);

        motor_LR = hardwareMap.get(DcMotor.class, "Motor_LR");
        motor_LR.setDirection(DcMotorSimple.Direction.REVERSE);

        pixel_Motor = hardwareMap.get(DcMotor.class, "pixel_Motor");
        pixel_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setAllMecanumPowers(0.0);
        pixel_Motor.setPower(0.0);

        telemetry.addLine("Begin initializations");
        telemetry.addLine("... Mecanum init complete");
        telemetry.addLine("End initializations");
    }

    @Override
    public void loop() {
        manualDrive();

        double armSpeedUp = 0.2;
        double armSpeedDown = 0.1;
        if (gamepad2.dpad_down) {
            pixel_Motor.setPower(armSpeedDown);
            pixel_Motor.setTargetPosition(0);
            pixel_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addLine("Moving Down");

        }
        if (gamepad2.dpad_up) {
            pixel_Motor.setPower(armSpeedUp);
            pixel_Motor.setTargetPosition(750);
            pixel_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addLine("Moving Up");
        }

        telemetry.update();
    }



    public void manualDrive() {
        double strafeSpeed = gamepad1.left_stick_x;
        double turnSpeed = gamepad1.right_stick_x;
        double driveSpeed = gamepad1.left_stick_y;

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
        motor_LF.setPower(driveSensitivity * LFpower);
        motor_RF.setPower(driveSensitivity * RFpower);
        motor_RR.setPower(driveSensitivity * RRpower);
        motor_LR.setPower(driveSensitivity * LRpower);
    }
}