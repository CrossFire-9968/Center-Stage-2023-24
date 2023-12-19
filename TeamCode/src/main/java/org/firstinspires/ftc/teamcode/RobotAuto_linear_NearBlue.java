package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Robot Auto Blue Near")
public class RobotAuto_linear_NearBlue extends LinearOpMode
{
    public DcMotor motor_LR;
    public DcMotor motor_RR;
    public DcMotor motor_LF;
    public DcMotor motor_RF;
    public  DcMotor Intake_Motor ;
    boolean isAutoComplete = false;
    Servo Ramp;

    int countsToDriveOneInch = 33;      // Approximate encoder counts to drive 1 inch
    int countsToRotate45Degrees = 450;  // Approximate encoder counts to rotate 45 degrees
    int countsToStrafeOneInch = 39;     // Approximate encoder counts to strafe 1 inch
    final double driveSensitivity = 0.7;
    double intakeMotorPower = -0.3;     // Intake motor power
    double pixelRampUp = 0.2;
    double pixelRampDown = 0.44;

    enum spikeLocation {
        LEFT, CENTER, RIGHT;
    };

    @Override
    public void runOpMode() throws InterruptedException
    {
        motor_LF = hardwareMap.get(DcMotor.class, "Motor_LF");
        motor_LF.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_RF = hardwareMap.get(DcMotor.class, "Motor_RF");
        motor_RF.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_RR = hardwareMap.get(DcMotor.class, "Motor_RR");
        motor_RR.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_LR = hardwareMap.get(DcMotor.class, "Motor_LR");
        motor_LR.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake_Motor = hardwareMap.get(DcMotor.class, "Intake_Motor");
        Intake_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        Ramp = hardwareMap.get(Servo.class, "Ramp");
        Ramp.setDirection((Servo.Direction.FORWARD));

        // Initialize robot
        initRobot();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Until we have camera working, this is a way to cycle pixel location for test
        // Hit a-button to cycle to position
        spikeLocation pixelLocation = spikeLocation.LEFT;
        String testLocation = "CENTER";
        if (gamepad1.start) {
            switch (pixelLocation) {
                case CENTER:
                    pixelLocation = spikeLocation.CENTER;
                    testLocation = "CENTER";
                    break;
                case RIGHT:
                    pixelLocation = spikeLocation.RIGHT;
                    testLocation = "RIGHT";
                    break;
                case LEFT:
                    pixelLocation = spikeLocation.LEFT;
                    testLocation = "LEFT";
                    break;
            }
        }

        telemetry.addData("Pixel Location: ", pixelLocation);

        // Run this code while Autonomous has not timed out
        while (opModeIsActive() && !isAutoComplete) {

//            // Right pixel location
//            if (pixelLocation == spikeLocation.RIGHT) {
//                dropRightPixel();
//            }
//            // Left pixel location
//            else if (pixelLocation == spikeLocation.LEFT) {
//                dropLeftPixel();
//            }
//            // Middle pixel location
//            else if (pixelLocation == spikeLocation.CENTER) {
//                dropCenterPixel();
//            }
//            // If position is not detectable, then just park
//            else {
//                // Need code for parking
//            }

            switch (pixelLocation) {
                // Center pixel location detected
                case CENTER:
                    dropCenterPixel();
                    break;

                // Right pixel location detected
                case RIGHT:
                    dropRightPixel();
                    break;

                // Left pixel location detected
                case LEFT:
                    dropLeftPixel();
                    break;

                // If position is not detectable, then just park
                default:
//                    simplePark();
//                    rotate();
            }


            // When auto is complete, code stops.
             isAutoComplete = true;
        }

        // After all the code runs once, Automomous is over so make robot safe and wait for teleop
        initRobot();
    }


    /**
     * <p>Initializes the robot to a de-energized state. Call this method before Autonomous
     * to prepare the robot and after Autonomous to shut down all the actuators.
     * </p>
     */
    public void initRobot() {
        // Turn off drivetrain motors
        setMecanumPowers(0.0);

        // Raise pixel ramp off floor and turn off intake motor
        Ramp.setPosition(pixelRampUp);
        Intake_Motor.setPower(0.0);
    }


    /**
     * <p> Sequence of events for dropping the pixel on the center tape and then parking </p>
     */
    public void dropCenterPixel() {
        double drivePower = 0.3;                // Motor power
        int driveDistanceFromWall = 32;         // Inches
        int driveDistanceToDropPixel = -28;     // Inches
        int strafeDistanceToPark = -48;         // Inches
        int rotateToPark = -875;
        int strafeToPark = -45;

        // Drive forward from wall
        drive(drivePower, driveDistanceFromWall * countsToDriveOneInch);
        waitForDriveToPosition();
        sleep(500);

        // Drop pixel - drop ramp, reverse intake, drive backwards
        Ramp.setPosition(pixelRampDown);
        Intake_Motor.setPower(intakeMotorPower);
        sleep(500);

        // Drive backwards to lay down pixel
        drive(drivePower, driveDistanceToDropPixel * countsToDriveOneInch);
        waitForDriveToPosition();

        // Turn off intake and bring up ramp
        Ramp.setPosition(pixelRampUp);
        Intake_Motor.setPower(0.0);
        sleep(500);

        // strafe to avoid bars
        strafe(drivePower, strafeToPark * countsToDriveOneInch);
        waitForDriveToPosition();
        sleep(500);

        // Rotate towards park position
        rotate(drivePower,rotateToPark);
        waitForDriveToPosition();
        sleep(500);




    }


    /**
     * <p> Sequence of events for dropping the pixel on the righthand tape and then parking </p>
     */
    public void dropRightPixel() {
        double drivePower = 0.3;            // Motor power
        int driveDistanceFromWall = 19;     // Inches
        int countsToRotateToPixel = 450;    // 450 is about 45 degrees
        int driveDistanceToTape = 9;  // Inches
        int driveDistanceToDropPixel = -27;  // Inches
        int countsToRotateToPark = -1200;    // 450 is about 45 degrees
        int driveDistanceToPark = 22;        // Inches

        // Drive forward from wall
        drive(drivePower, driveDistanceFromWall * countsToDriveOneInch);
        waitForDriveToPosition();
        sleep(500);

        // Rotate towards tape
        rotate(drivePower, countsToRotateToPixel);
        waitForDriveToPosition();
        sleep(500);

        // Drive forward to tape
        drive(drivePower, driveDistanceToTape * countsToDriveOneInch);
        waitForDriveToPosition();
        sleep(500);

        // Drop pixel - drop ramp, reverse intake, drive backwards
        Ramp.setPosition(pixelRampDown);
        Intake_Motor.setPower(intakeMotorPower);
        sleep(500);

        // Drive backwards to lay down pixel
        drive(drivePower, driveDistanceToDropPixel * countsToDriveOneInch);
        waitForDriveToPosition();

        // Turn off intake and bring up ramp
        Ramp.setPosition(pixelRampUp);
        Intake_Motor.setPower(0.0);
        sleep(500);

        // Rotate toward backdrop
        rotate(drivePower, countsToRotateToPark);
        waitForDriveToPosition();
        sleep(500);

        // Rotate and Drive to park position
        drive(drivePower, driveDistanceToPark * countsToDriveOneInch);
        waitForDriveToPosition();
    }


    /**
     * <p> Sequence of events for dropping the pixel on the lefthand tape and then parking </p>
     */
    public void dropLeftPixel(){
        double drivePower = 0.3;            // Motor power
        int driveDistanceFromWall = 24;     // Inches
        int countsToRotateToPixel = -450;    // 450 is about 45 degrees
        int driveDistanceToTape = 8;  // Inches
        int driveDistanceToDropPixel = -18;  // Inches
        int countsToRotateToPark = -700;    // 450 is about 45 degrees
        int driveDistanceToPark = 22;        // Inches

        // Drive forward from wall
        drive(drivePower, driveDistanceFromWall * countsToDriveOneInch);
        waitForDriveToPosition();
        sleep(500);

        //Strafe
        strafe(drivePower,-13 * countsToDriveOneInch);
        waitForDriveToPosition();
        sleep(500);


        // Drop pixel - drop ramp, reverse intake, drive backwards
        Ramp.setPosition(pixelRampDown);
        Intake_Motor.setPower(intakeMotorPower);
        sleep(500);

        // Drive backwards to lay down pixel
        drive(drivePower, driveDistanceToDropPixel * countsToDriveOneInch);
        waitForDriveToPosition();

        // Turn off intake and bring up ramp
        Ramp.setPosition(pixelRampUp);
        Intake_Motor.setPower(0.0);
        sleep(500);

        // Rotate toward backdrop
        rotate(drivePower, countsToRotateToPark);
        waitForDriveToPosition();
        sleep(500);

        // Drive to park position
        drive(drivePower, driveDistanceToPark * countsToDriveOneInch);
        waitForDriveToPosition();
    }


    /**
     * <p> Sequence of events for dropping the pixel on the lefthand tape and then parking </p>
     */
    public void drive(double power, int distance) {
        stopAndResetEncoders();
        setMecanumPowers(power);
        setTargetPosition(distance);
        runToPosition();
    }


    /**
     * <p> Sequence of events for dropping the pixel on the lefthand tape and then parking </p>
     */
    public void strafe (double power, int distance)
    {
        stopAndResetEncoders();
        setMecanumPowers(power);
        setTargetPosition(distance, -distance, distance, -distance);
        runToPosition();
    }


    /**
     * <p> Sequence of events for dropping the pixel on the lefthand tape and then parking </p>
     */
    public void rotate (double power, int counts) {
        stopAndResetEncoders();
        setMecanumPowers(power);
        setTargetPosition(counts, -counts, -counts, counts);
        runToPosition();
    }


    /**
     * <p> Sequence of events for dropping the pixel on the lefthand tape and then parking </p>
     */
    protected void setMecanumPowers(double power) {
        motor_LF.setPower(power);
        motor_RF.setPower(power);
        motor_RR.setPower(power);
        motor_LR.setPower(power);
    }


    /**
     * <p> Drive until one of the 4 wheels has reached it's target position. We only wait for one
     * because it is not guaranteed all 4 wheels will reach their target at the same time due
     * to inconsistencies in alignment and resistance in the drivetrain. If we wait for all 4 wheels
     * They will start fighting one another in a tug-of-war type effect and we may not transition
     * to the next stage as we expect.
     * </p>
     */
    public void waitForDriveToPosition() {
        while (motor_LF.isBusy() && motor_RF.isBusy() && motor_RR.isBusy() && motor_LR.isBusy())
        {
            idle();
        }

        // Once position is reached stop all motors and reset their relative encoder counts to zero
        stopAndResetEncoders();
    }


    /**
     * <p> Turn off all drivetrain motors and reset encoder counts </p>
     */
    private void stopAndResetEncoders() {
        motor_LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    /**
     * <p> Commands all drivetrain motors to start motion </p>
     */
    private void runToPosition() {
        motor_LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    /**
     * <p> Sets target position for all drivetrain motors each to a custom value</p>
     */
    private void setTargetPosition(int LF_distance, int RF_distance, int RR_distance, int LR_distance)
    {
        motor_LF.setTargetPosition(LF_distance);
        motor_RF.setTargetPosition(RF_distance);
        motor_RR.setTargetPosition(RR_distance);
        motor_LR.setTargetPosition(LR_distance);
    }


    /**
     * <p> Sets target position for all drivetrain motors to same value </p>
     */
    private void setTargetPosition(int distance)
    {
        motor_LF.setTargetPosition(distance);
        motor_RF.setTargetPosition(distance);
        motor_RR.setTargetPosition(distance);
        motor_LR.setTargetPosition(distance);
    }
}