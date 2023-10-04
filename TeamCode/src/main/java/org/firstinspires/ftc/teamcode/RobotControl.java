package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Robot TeleOp")
public class RobotControl extends OpMode {
    private DcMotor MotorLRear;
    private DcMotor MotorRRear;
    private DcMotor MotorLFront;
    private DcMotor MotorRFront;
    double driveSpeed;
    double turnSpeed;

    @Override
    public void init( ) {
        MotorLRear = hardwareMap.get(DcMotor.class, "MotorLRear");
        MotorLRear.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorLRear.setPower(0.0);

        MotorRRear = hardwareMap.get(DcMotor.class, "MotorRRear");
        MotorRRear.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorRRear.setPower(0.0);

        MotorLFront = hardwareMap.get(DcMotor.class, "MotorLFront");
        MotorLFront.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorLFront.setPower(0.0);

        MotorRFront = hardwareMap.get(DcMotor.class, "MotorRFront");
        MotorRFront.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorRFront.setPower(0.0);



    }


    @Override
    public void loop() {
        turnSpeed = gamepad1.right_stick_x;
        driveSpeed = -gamepad1.left_stick_y;
        MotorRFront.setPower(driveSpeed);
        MotorLFront.setPower(driveSpeed);
        MotorRRear.setPower(driveSpeed);
        MotorLRear.setPower(driveSpeed);

        MotorRFront.setPower(-turnSpeed);
        MotorLFront.setPower(turnSpeed);
        MotorRRear.setPower(-turnSpeed);
        MotorLRear.setPower(turnSpeed);

        telemetry.addData("speed Forward", driveSpeed);
        telemetry.update();
    }
}

