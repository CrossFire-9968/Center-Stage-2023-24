package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.text.DecimalFormat;

@TeleOp(name="Jeff Test")
public class RobotControl extends OpMode {
   enum DriveMode {
      MANUAL,
      ENCODERS;
   }

   Servo testServo = null;
   DcMotor testMotor = null;
   DriveMode driveMode = DriveMode.MANUAL;
   int targetPosition = 0;


   /**
    * <p>
    * This method will be called once, when the INIT button is pressed.
    */
   @Override
   public void init() {
      testServo = hardwareMap.get(Servo.class, "Test Servo");
      testMotor = hardwareMap.get(DcMotor.class, "Test Motor");
      testMotor.setDirection(DcMotorSimple.Direction.REVERSE);
   }

   /**
    * <p>
    * This method will be called repeatedly during the period between when
    * the init button is pressed and when the play button is pressed (or the
    * OpMode is stopped).
    * <p>
    * This method is optional. By default, this method takes no action.
    */
   @Override
   public void init_loop() {}

   /**
    * <p>
    * This method will be called once, when the play button is pressed.
    * <p>
    * This method is optional. By default, this method takes no action.
    * <p>
    * Example usage: Starting another thread.
    */
   @Override
   public void start() {}

   /**
    * <p>
    * This method will be called repeatedly during the period between when
    * the play button is pressed and when the OpMode is stopped.
    */
   @Override
   public void loop() {
      // Control the servo
      if (gamepad1.dpad_left) {
         testServo.setPosition(0.2);
      }

      if (gamepad1.dpad_right) {
         testServo.setPosition(0.7);
      }

      telemetry.addData("Servo Position: ", testServo.getPosition());

      // Control the motor
      if (gamepad1.a) {
         driveMode = DriveMode.MANUAL;
      }

      if (gamepad1.b) {
         driveMode = DriveMode.ENCODERS;
         testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      }

      if (driveMode == DriveMode.MANUAL) {
         this.driveManual();
      }

      if (driveMode == DriveMode.ENCODERS) {
         this.driveByEncoders();
      }

      telemetry.addData("Motor Power: ", testMotor.getPower());
      telemetry.addData("Motor Position:", testMotor.getCurrentPosition());
      telemetry.addData("Motor Target Position:", testMotor.getTargetPosition());
      telemetry.update();
   }

   /**
    * <p>
    * This method will be called once, when this OpMode is stopped.
    * <p>
    * Your ability to control hardware from this method will be limited.
    * <p>
    * This method is optional. By default, this method takes no action.
    */
   @Override
   public void stop() {}

   public void driveManual() {
      testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      testMotor.setPower(gamepad1.left_stick_y);

      telemetry.addLine("Driving Manual");
   }

   public void driveByEncoders() {
      if (gamepad1.x) {
         targetPosition = 300;
      }
      else if (gamepad1.y) {
         targetPosition = -300;
      }

      testMotor.setPower(1.0);
      testMotor.setTargetPosition(targetPosition);
      testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
   }
}
