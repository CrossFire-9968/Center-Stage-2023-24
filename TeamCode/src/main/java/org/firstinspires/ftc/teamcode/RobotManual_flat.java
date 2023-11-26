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
    public DcMotor Intake_Motor;
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
    public Servo Launcher;
    public Servo Ramp;
    final double driveSensitivity = 0.7;
    double bucketDownPosition = 0.85;
    double bucketUpPosition = 0.05;
    double bucketDumpPosition = 0.3;
    double armSpeedUp = 0.2;
    double armSpeedDown = 0.1;
    int pixelArmCountsUp = -1330;
    int pixelArmCountsDown = 0;
    double launcherMin = 0.4;
    double launcherMax = 1.0;
    static final double RAISEINCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double DUMPINCREMENT   = 0.005;     // amount to slew servo each CYCLE_MS cycle

    static final int    bucketDelay_MS    =   20;     // period of each cycle
    boolean setArmMoving = false;
    double bucketPosition = 0.0;
    private static ElapsedTime bucketTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    boolean useBucketTimer = false;
    boolean wasAPressed = false;
    boolean wasYPressed = false;
    boolean movingArmDown = false;
    boolean movingArmUp = false;
    boolean movingToIncrement = false;
    double pixelRampDown = 0.44;
    double pixelRampUp = 0.2;
    double intakePowerMin = 0.1;
    double outtakePowerMin = -0.1;
    double pixelIntakePower = 0.95;
    double pixelOuttakePower = -0.6;
    double strafeMax = 1.0;


    enum bucketDestination {
        DOWN, UP, BUCKETDOWN, BUCKETUP;
    };

    @Override
    public void init() {
        Intake_Motor = hardwareMap.get(DcMotor.class, "Intake_Motor");
        Intake_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

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
        bucket.setPosition(bucketDownPosition);
        bucketPosition = bucketDownPosition;

        Launcher = hardwareMap.get(Servo.class, "Launcher");
        Launcher.setDirection(Servo.Direction.FORWARD);
        Launcher.setPosition(launcherMin);

        Ramp = hardwareMap.get(Servo.class, "Ramp");
        Ramp.setDirection((Servo.Direction.FORWARD));
        Ramp.setPosition(pixelRampUp);

        setAllMecanumPowers(0.0);
        pixel_Motor.setPower(0.0);

        telemetry.addLine("End of initializations");
        telemetry.update();
    }

    @Override
    public void loop() {
        manualDrive();
        pixelArmControl();
        launchDrone();
        runIntake();
        telemetry.addData("bucketPosition: ", bucket.getPosition());
        telemetry.addData("bucket timer", bucketTimer.time());
        telemetry.addData("PixelArm", pixel_Motor.getCurrentPosition());
        telemetry.update();

    }

    public void launchDrone() {
        if (gamepad2.right_bumper) {
            Launcher.setPosition(launcherMax);
        }
    }

    public void runIntake () {
        double intakePower = -gamepad2.left_stick_y;

        intakePower = Range.clip(intakePower, pixelOuttakePower, pixelIntakePower);

        // Bring pixel into bucket
        if (intakePower >= intakePowerMin) {
            Intake_Motor.setPower(intakePower);
            Ramp.setPosition(pixelRampDown);
        }
        // Expel pixel out of robot
        else if (intakePower <= outtakePowerMin) {
            Intake_Motor.setPower(intakePower);
            Ramp.setPosition(pixelRampDown);
        }
        else {
            Intake_Motor.setPower(0.0);
            Ramp.setPosition(pixelRampUp);
        }

        telemetry.addData("intakePower", intakePower);


    }



    public void manualDrive() {
        double turnSpeed = gamepad1.right_stick_x;
        double driveSpeed = gamepad1.left_stick_y;
        double strafeSpeed = 0.0;

        if (gamepad1.left_bumper) {
             strafeSpeed = -strafeMax;
        }
        else if (gamepad1.right_bumper) {
            strafeSpeed =  strafeMax;
        }

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
            movingArmDown = true;
            movingArmUp = false;
            pixel_Motor.setPower(armSpeedDown);
            pixel_Motor.setTargetPosition(pixelArmCountsDown);
            pixel_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (gamepad2.y && !wasYPressed) {
            movingArmUp = true;
            movingArmDown = false;
            pixel_Motor.setPower(armSpeedUp);
            pixel_Motor.setTargetPosition(pixelArmCountsUp);
            pixel_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
//        if(gamepad2.b)
//        {
//            bucket.setPosition(0.5);
//        }


        // When you press gamepad input the arm goes to up position.


        // Code called each loop make the bucket move to the next increment in rotation
        // The code has a timer so we can slow down how fast the bucket servo rotates.
        if (movingArmDown) {
            rotateBucketToPosition(bucketDestination.DOWN);
        }
        else if (movingArmUp) {
            rotateBucketToPosition(bucketDestination.UP);
        }
        else if (gamepad2.b) {
            rotateBucketToPosition(bucketDestination.BUCKETDOWN);
        }
        else if (gamepad2.x) {
            rotateBucketToPosition(bucketDestination.BUCKETUP);
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
                case DOWN:
                    bucketPosition += RAISEINCREMENT;
                    if (bucketPosition >= bucketDownPosition) {
                        movingArmDown = false;
                        movingToIncrement = false;
                    }
                    break;
                case UP:
                    bucketPosition -= RAISEINCREMENT;
                    if (bucketPosition <= bucketUpPosition) {
                        movingArmUp = false;
                        movingToIncrement = false;
                    }
                    break;
                case BUCKETDOWN:
                    if (bucketPosition <= bucketDumpPosition) {
                        bucketPosition += DUMPINCREMENT;
                    }
                    break;
                case BUCKETUP:
                    if (bucketPosition >= bucketUpPosition) {
                        bucketPosition -= DUMPINCREMENT;
                    }
                    break;
            }

            telemetry.addData("bucketPosition: ", bucketUpPosition);

            // Only move to the next increment in position if the bucket hasn't reached the desired position
            if (movingToIncrement)
            {
                Range.clip(bucketPosition, bucketDumpPosition, bucketDownPosition);
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
