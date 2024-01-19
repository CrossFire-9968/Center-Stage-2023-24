package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class GripperArm {
    public Servo gripper;
    public TouchSensor gripperTouchUpper;
    public TouchSensor gripperTouchLower;
    int gripperArmPixelPosition = 500;
    int gripperArmHomePosition = 0;
    int gripperArmHangPosition = 700;
    double openGripperValue = 0.9;
    double closedGripperValue = 0.45;
    public CRServo armExtender;
    public TouchSensor armExtenderLimit;
    double armSpeedUp = 0.4;
    double armSpeedDown = 0.2;
    public DcMotor pixel_Motor;

    public void init(HardwareMap hwMap){
        gripperTouchUpper = hwMap.get(TouchSensor.class, "gripper_Touch_Upper");
        gripperTouchLower = hwMap.get(TouchSensor.class, "gripper_Touch_Lower");

        armExtender = hwMap.get(CRServo.class, "arm_extend_servo");
        armExtender.setDirection(CRServo.Direction.FORWARD);
        armExtender.setPower(0.0);

        gripper = hwMap.get(Servo.class, "Gripper");
        gripper.setDirection((Servo.Direction.FORWARD));
        gripper.setPosition(closedGripperValue);

        armExtenderLimit = hwMap.get(TouchSensor.class, "arm_limit");
    }

    public void triggerControl(Gamepad gamepad) {
        double zeroThreshold = 0.1;

        if (gamepad.left_trigger > zeroThreshold) {
            gripper.setPosition(openGripperValue);
        }
        else if (gamepad.right_trigger > zeroThreshold) {
            gripper.setPosition(closedGripperValue);
        }
    }

    protected void armControl(Gamepad gamepad) {
        double threshold = 0.1;
        double armExtenderMaxPower = 1.0;

        // Kicks off the bucket rotation but only when the button is first switches
        // from unpressed (false) to pressed (true).
        if (gamepad.a) {
            pixel_Motor.setPower(armSpeedDown);
            pixel_Motor.setTargetPosition(gripperArmHomePosition);
            pixel_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (gamepad.y) {
            pixel_Motor.setPower(armSpeedUp);
            pixel_Motor.setTargetPosition(gripperArmPixelPosition);
            pixel_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (gamepad.x) {
            pixel_Motor.setPower(armSpeedUp);
            pixel_Motor.setTargetPosition(gripperArmHangPosition);
            pixel_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad.left_stick_y > threshold) {
            armExtender.setPower(armExtenderMaxPower);
        }
        else if (gamepad.left_stick_y < -threshold) {
            armExtender.setPower(-armExtenderMaxPower);
        }
        else
            armExtender.setPower(0.0);
    }
}
