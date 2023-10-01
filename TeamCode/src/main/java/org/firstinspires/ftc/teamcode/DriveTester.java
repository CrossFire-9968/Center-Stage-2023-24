package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp()
public class DriveTester extends OpMode {
    private DcMotor MotorLRear;
    private DcMotor MotorRRear;
    private DcMotor MotorLFront;
    private DcMotor MotorRFront;
    double speedForward;

    @Override
    public void init( ) {
        MotorLRear = hardwareMap.get(DcMotor.class, "MotorLRear");
        MotorRRear = hardwareMap.get(DcMotor.class, "MotorRRear");
        MotorLFront = hardwareMap.get(DcMotor.class, "MotorLFront");
        MotorRFront = hardwareMap.get(DcMotor.class, "MotorRFront");
        MotorRRear.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorLFront.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorRFront.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorLRear.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    @Override
    public void loop() {
        speedForward = -gamepad1.left_stick_y ;
        MotorLRear.setPower(speedForward);
        MotorRRear.setPower(speedForward);
        MotorLFront.setPower(speedForward);
        MotorRFront.setPower(speedForward);
        telemetry.addData("speed Forward", speedForward);
        telemetry.update();


    }
}

