package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Robot Manual Flat")
public class RobotManual extends OpMode {
    public DroneLauncher drone;
    public DcMotor motor_LR;
    public DcMotor motor_RR;
    public DcMotor motor_LF;
    public DcMotor motor_RF;
    public DcMotor pixel_Motor;
    public TouchSensor armExtenderLimit;
    double LFrontPower;
    double RFrontPower;
    double RRearPower;
    double LRearPower;
    public Servo gripper;
    public TouchSensor gripperTouchUpper;
    public TouchSensor gripperTouchLower;
    final double driveSensitivity = 0.7;
    double armSpeedUp = 0.4;
    double armSpeedDown = 0.2;
    int gripperArmPixelPosition = 500;
    int gripperArmHomePosition = 0;
    int gripperArmHangPosition = 700;
    double openGripperValue = 0.9;
    double closedGripperValue = 0.45;
    double strafeMax = 1.0;
    public DcMotor Hanger_Motor1;
    public DcMotor Hanger_Motor2;
    public CRServo armExtender;
    public RevBlinkinLedDriver blinkin;
    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    public boolean playWasPressed = false;

    @Override
    public void init() {
        drone.init(hardwareMap);

        motor_LF = hardwareMap.get(DcMotor.class, "Motor_LF");
        motor_LF.setDirection(DcMotorSimple.Direction.FORWARD);

        motor_RF = hardwareMap.get(DcMotor.class, "Motor_RF");
        motor_RF.setDirection(DcMotorSimple.Direction.REVERSE);

        motor_RR = hardwareMap.get(DcMotor.class, "Motor_RR");
        motor_RR.setDirection(DcMotorSimple.Direction.FORWARD);

        motor_LR = hardwareMap.get(DcMotor.class, "Motor_LR");
        motor_LR.setDirection(DcMotorSimple.Direction.REVERSE);

        gripperTouchUpper = hardwareMap.get(TouchSensor.class, "gripper_Touch_Upper");
        gripperTouchLower = hardwareMap.get(TouchSensor.class, "gripper_Touch_Lower");

        Hanger_Motor1 = hardwareMap.get(DcMotor.class, "Hanger_Motor1");
        Hanger_Motor1.setDirection(DcMotorSimple.Direction.FORWARD);

        Hanger_Motor2 = hardwareMap.get(DcMotor.class, "Hanger_Motor2");
        Hanger_Motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        pixel_Motor = hardwareMap.get(DcMotor.class, "pixel_Motor");
        pixel_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        pixel_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armExtender = hardwareMap.get(CRServo.class, "arm_extend_servo");
        armExtender.setDirection(CRServo.Direction.FORWARD);
        armExtender.setPower(0.0);

        gripper = hardwareMap.get(Servo.class, "Gripper");
        gripper.setDirection((Servo.Direction.FORWARD));
        gripper.setPosition(closedGripperValue);

        armExtenderLimit = hardwareMap.get(TouchSensor.class, "arm_limit");

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        setAllMecanumPowers(0.0);
        pixel_Motor.setPower(0.0);
        Hanger_Motor1.setPower(0.0);
        Hanger_Motor2.setPower(0.0);

        telemetry.addLine("End of initializations");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (!playWasPressed) {
            timer.reset();
            playWasPressed = true;
        }

//        if (armExtenderLimit.isPressed()) {
//            telemetry.addLine("Sensor On");
//        }
//        else if (!armExtenderLimit.isPressed()) {
//            telemetry.addLine("Sensor Off");
//        }

        if (gripperTouchUpper.isPressed()) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);        }
        else if (gripperTouchLower.isPressed()) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        else if (timer.seconds() >= 90 && timer.seconds() < 120) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
        }
        else if (timer.seconds() >= 120) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        }
        else {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }

        telemetry.addData("Time: ", timer.seconds());

        // The following methods are called iteratively, over and over again
        // Instead of putting all the code in loop(), we break it up into methods
        // to make the code easier to maintain. As we advance in our coding, we'll
        // put these into different classes.

        drone.control(gamepad2);

        manualDrive();      // Operates the mechanum drive motors
        gripperArmControl();  // Operates the pixel arm motor and bucket servo
        gripperControl();        // Operates the intake motor and ramp
        hangerControl();    // Operates the hanger motors and servo for lifting the robot

        // Stuff we want to see during game play
        telemetry.addData("PixelArm", pixel_Motor.getCurrentPosition());
        telemetry.update();
    }

    /**
     * <p> Method operates the drivetrain motors </p>
     */
    public void manualDrive() {
        double turnSpeed = gamepad1.right_stick_x;
        double driveSpeed = gamepad1.left_stick_y;
        double strafeSpeed = 0.0;

        if (gamepad1.left_bumper) {
            strafeSpeed = -strafeMax;
        }
        else if (gamepad1.right_bumper) {
            strafeSpeed = strafeMax;
        }

        // Raw drive power for each motor from joystick inputs
        LFrontPower = driveSpeed - turnSpeed - strafeSpeed;
        RFrontPower = driveSpeed + turnSpeed + strafeSpeed;
        RRearPower = driveSpeed + turnSpeed - strafeSpeed;
        LRearPower = driveSpeed - turnSpeed + strafeSpeed;

        //        // Cubing power values to give finer control at slow speeds
//      LFrontPower = Math.pow(LFrontPower, 1);
//      RFrontPower = Math.pow(RFrontPower, 1);
//      RRearPower = Math.pow(RRearPower, 1);
//      LRearPower = Math.pow(LRearPower, 1);

        double max = 1.0;
        max = Math.max(max, Math.abs(LFrontPower));
        max = Math.max(max, Math.abs(RFrontPower));
        max = Math.max(max, Math.abs(RRearPower));
        max = Math.max(max, Math.abs(LRearPower));

        // Ratio drive powers
        LFrontPower = (LFrontPower / max);
        RFrontPower = (RFrontPower / max);
        RRearPower = (RRearPower / max);
        LRearPower = (LRearPower / max);

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


    public void gripperControl() {
        double zeroThreshold = 0.1;

        if (gamepad2.left_trigger > zeroThreshold) {
            gripper.setPosition(openGripperValue);
        }
        else if (gamepad2.right_trigger > zeroThreshold) {
            gripper.setPosition(closedGripperValue);
        }
    }


    protected void gripperArmControl() {
        double threshold = 0.1;
        double armExtenderMaxPower = 1.0;

        // Kicks off the bucket rotation but only when the button is first switches
        // from unpressed (false) to pressed (true).
        if (gamepad2.a) {
            pixel_Motor.setPower(armSpeedDown);
            pixel_Motor.setTargetPosition(gripperArmHomePosition);
            pixel_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (gamepad2.y) {
            pixel_Motor.setPower(armSpeedUp);
            pixel_Motor.setTargetPosition(gripperArmPixelPosition);
            pixel_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (gamepad2.x) {
            pixel_Motor.setPower(armSpeedUp);
            pixel_Motor.setTargetPosition(gripperArmHangPosition);
            pixel_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad2.left_stick_y > threshold) {
            armExtender.setPower(armExtenderMaxPower);
        }
        else if (gamepad2.left_stick_y < -threshold) {
            armExtender.setPower(-armExtenderMaxPower);
        }
        else
            armExtender.setPower(0.0);
    }

    /**
     * <p>Method operates the hanger system which uses a servo to deploy the lift cable and
     * two motors to winch the robot to a hanging position.</p>
     */

    public void hangerControl() {
        // Winch the robot up off the floor
        if (gamepad2.dpad_right) {
            Hanger_Motor1.setPower(-1.0);
            Hanger_Motor2.setPower(1.0);
        }

        // Lower the robot toward the floor
        else if (gamepad2.dpad_left) {
            Hanger_Motor1.setPower(1.0);
            Hanger_Motor2.setPower(-1.0);
        }
        else {
            Hanger_Motor1.setPower(0.0);
            Hanger_Motor2.setPower(0.0);
        }
    }
}
