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
    double LFrontPower;
    double RFrontPower;
    double RRearPower;
    double LRearPower;
    double DriveSpeed = 0.5;
    int DriveDistance = 33;
    int BackDriveDistance = -28;
    //-28
    int DriveCounts = 32;
    int TurnDegrees = 800;
    final double driveSensitivity = 0.7;
    boolean isAutoComplete= false;
    double pixelRampUp = 0.2;
    double pixelRampDown = 0.44;
    public  DcMotor Intake_Motor ;
    Servo Ramp;

    enum spikeLocation {
        LEFT, MIDDLE, RIGHT;
    };

    @Override
    public void runOpMode() throws InterruptedException
    {
        spikeLocation pixelLocation = spikeLocation.MIDDLE;

        // Declare any local / helper variables here
        // Our initialization code should go here before calling "WaitForStart()"


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
        Ramp.setPosition(pixelRampUp);

        setMecanumPowers(0.0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run this code while Autonomous has not timed out
        while (opModeIsActive() && !isAutoComplete) {
            drive(DriveSpeed, DriveDistance * DriveCounts);

            while(!isMotionComplete()){
                idle();
            }

            // Turn toward pixel location
            if (pixelLocation == spikeLocation.RIGHT){
                rotate(DriveSpeed, TurnDegrees);
                
                while(!isMotionComplete()){
                    idle();
                }
                sleep(1000);
            }
            else if (pixelLocation == spikeLocation.LEFT){
                rotate(DriveSpeed, -TurnDegrees);

                while(!isMotionComplete()){
                    idle();
                }
                sleep(1000);
            }



            Ramp.setPosition(pixelRampDown);
            Intake_Motor.setPower(-0.3);
            drive(DriveSpeed, BackDriveDistance * DriveCounts);

            while(!isMotionComplete()){
                idle();}

            sleep(1000);
            Intake_Motor.setPower(0.0);


            Ramp.setPosition(pixelRampUp);
            strafe(0.5, -48*40);

            while(!isMotionComplete()){
                idle();}

            isAutoComplete = true;
        }

    }

// 32 counts per inch est.
    public void drive(double power, int distance) {
        stopAndResetEncoders();
        setMecanumPowers(power);
        setTargetPosition(distance);
        runToPosition();
    }

// 39 counts per inch est.
    public void strafe (double power, int distance)
    {
        stopAndResetEncoders();
        setMecanumPowers(power);
        setTargetPosition(distance, -distance, distance, -distance);
        runToPosition();
    }


    public void rotate (double power, int distance) {
        stopAndResetEncoders();
        setMecanumPowers(power);
        setTargetPosition(distance, -distance, -distance, distance);
        runToPosition();
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


    // Drive until one of the 4 wheels has reached it's target position. We only wait for one
    // because it is not guaranteed all 4 wheels will reach their target at the same time due
    // to inconsistencies in alignment and resistance in the drivetrain. If we wait for all 4 wheels
    // They will start fighting one another in a tug-of-war type effect and we may not transition
    // to the next stage as we expect.
    public boolean isMotionComplete() {
        boolean motionComplete = false;

        // Only watch until one of the wheels reaches position. At that point, stop all the motors
        if (!motor_LF.isBusy() || !motor_RF.isBusy() || !motor_RR.isBusy() || !motor_LR.isBusy()) {
            stopAndResetEncoders();
            motionComplete = true;
        }

        return (motionComplete);
    }


    private void stopAndResetEncoders() {
        motor_LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    private void runToPosition() {
        motor_LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private void setTargetPosition(int LF_distance, int RF_distance, int RR_distance, int LR_distance)
    {
        motor_LF.setTargetPosition(LF_distance);
        motor_RF.setTargetPosition(RF_distance);
        motor_RR.setTargetPosition(RR_distance);
        motor_LR.setTargetPosition(LR_distance);
    }

    private void setTargetPosition(int distance)
    {
        motor_LF.setTargetPosition(distance);
        motor_RF.setTargetPosition(distance);
        motor_RR.setTargetPosition(distance);
        motor_LR.setTargetPosition(distance);
    }

    public void endOfAutoShutdown() {
        stopAndResetEncoders();
    }
}