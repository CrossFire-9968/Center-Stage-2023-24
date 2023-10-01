package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp()
public class GamePadTester extends OpMode {
    private DcMotor motor;

    @Override
    public void init( ) {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void loop() {
        double speedForward = -gamepad1.left_stick_y / 2.0;
        motor.setPower(speedForward);
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("speed Forward", speedForward);
        telemetry.update();


    }
}

