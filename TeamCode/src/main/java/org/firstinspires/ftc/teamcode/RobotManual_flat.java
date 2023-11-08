package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Robot Manual Flat")
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
    public Servo bucket;
    final double driveSensitivity = 0.7;
    double bucketRampPosition = 0.85;
    double bucketDumpPosition = 0.0;
    double armSpeedUp = 0.2;
    double armSpeedDown = 0.1;
    int pixelArmCountsUp = -1330;
    int pixelArmCountsDown = 0;
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    bucketDelay_MS    =   20;     // period of each cycle
    boolean setArmMoving = false;
    double bucketPosition = 0.0;
    private static ElapsedTime bucketTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    boolean useBucketTimer = false;
    boolean wasAPressed = false;
    boolean wasYPressed = false;
    boolean movingToRamp = false;
    boolean movingToDump = false;
    boolean movingToIncrement = false;

    enum bucketDestination {
        RAMP, DUMP;
    };

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

        bucket = hardwareMap.get(Servo.class, "bucket_Servo");
        bucket.setDirection(Servo.Direction.FORWARD);
        bucket.setPosition(bucketRampPosition);
        bucketPosition = bucketRampPosition;

        setAllMecanumPowers(0.0);
        pixel_Motor.setPower(0.0);

        telemetry.addLine("End of initializations");
        telemetry.update();
    }

    @Override
    public void loop() {
        manualDrive();
        pixelArmControl();

        telemetry.addData("bucketPosition: ", bucket.getPosition());
        telemetry.addData("bucket timer", bucketTimer.time());
        telemetry.addData("PixelArm", pixel_Motor.getCurrentPosition());
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


    protected void pixelArmControl()
    {
        // Kicks off the bucket rotation but only when the button is first switches
        // from unpressed (false) to pressed (true).
        if(gamepad2.a && !wasAPressed) {
            movingToRamp = true;
            movingToDump = false;
            pixel_Motor.setPower(armSpeedDown);
            pixel_Motor.setTargetPosition(pixelArmCountsDown);
            pixel_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (gamepad2.y && !wasYPressed) {
            movingToDump = true;
            movingToRamp = false;
            pixel_Motor.setPower(armSpeedUp);
            pixel_Motor.setTargetPosition(pixelArmCountsUp);
            pixel_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(gamepad2.b)
        {
            bucket.setPosition(0.5);
        }


        // When you press gamepad input the arm goes to up position.


        // Code called each loop make the bucket move to the next increment in rotation
        // The code has a timer so we can slow down how fast the bucket servo rotates.
        if (movingToRamp) {
            rotateBucketToPosition(bucketDestination.RAMP);
        }
        else if (movingToDump) {
            rotateBucketToPosition(bucketDestination.DUMP);
        }

        // Retain the last state of the bucket rotation input so we can use it for assessing
        // the transition from false to true (on press). You could also use it for determining
        // on release transitions if you want as well.
        wasAPressed = gamepad2.a;
        wasYPressed = gamepad2.y;
    }


    private void rotateBucketToPosition(bucketDestination destination)
    {
        if (!movingToIncrement)
        {
            // Initially set this to true, but later if we find the bucket is at the travel limit
            // set it to false as we have already reached the desire destination.
            movingToIncrement = true;

            // Increment or decrement the bucket position based on if we commanded
            // it to rotate to the ramp or dump positions.
            switch (destination) {
                case RAMP:
                    bucketPosition += INCREMENT;
                    if (bucketPosition >= bucketRampPosition) {
                        movingToRamp = false;
                        movingToIncrement = false;
                    }
                    break;
                case DUMP:
                    bucketPosition -= INCREMENT;
                    if (bucketPosition <= bucketDumpPosition) {
                        movingToDump = false;
                        movingToIncrement = false;
                    }
                    break;
            }

            // Only move to the next increment in position if the bucket hasn't reached the desired position
            if (movingToIncrement)
            {
                Range.clip(bucketPosition, bucketDumpPosition, bucketRampPosition);
                bucket.setPosition(bucketPosition);
                bucketTimer.reset();
            }
        }
        // Wait to rotate the servo to the next increment for teh desired amount of time. The longer the
        // delay between increments, the slower the bucket will rotate.
        else if (bucketTimer.time() >= bucketDelay_MS) {
            movingToIncrement = false;
        }
    }
}
