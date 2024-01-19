package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class GripperArm {
    public Servo gripper;
    public TouchSensor gripperTouchUpper;
    public TouchSensor gripperTouchLower;
    final double driveSensitivity = 0.7;
    int gripperArmPixelPosition = 500;
    int gripperArmHomePosition = 0;
    int gripperArmHangPosition = 700;
    double openGripperValue = 0.9;
    double closedGripperValue = 0.45;
    public CRServo armExtender;
    public TouchSensor armExtenderLimit;
    double armSpeedUp = 0.4;
    double armSpeedDown = 0.2;

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
}
