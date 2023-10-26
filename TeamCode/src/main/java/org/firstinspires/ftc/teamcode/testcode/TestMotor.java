package org.firstinspires.ftc.teamcode.testcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
   This is an test item for testing type DcMotor.
*/
public class TestMotor extends TestItem
{
   private double testMotorPower;
   private DcMotor testMotor;
   private Telemetry testTelemetry;
   private double motorPower;
   private boolean runTest;

   /**
    * <p>Class used for adding a DcMotor as a testable item</p>
    * @param description Label shown on display
    * @param motor DcMotor object to test
    */
   public TestMotor(String description, DcMotor motor)
   {
      super(description);
      testMotor = motor;
   }

   /**
    * <p>Class defining actions to perform during test</p>
    * @param runTest If true, selected test begins
    * @param telemetry Telemetry object to display on driver station during test
    * @param gamepad Gamepad currently in use for test, allowing for additional control of test hardware
    */
   @Override
   public void run(boolean runTest, Telemetry telemetry, Gamepad gamepad)
   {
      this.motorPower = gamepad.left_stick_y;
      this.runTest = gamepad.a;
      this.testTelemetry = telemetry;

      // During test, left y-joystick used to proportionally control motor
      if (runTest) {
         testMotor.setPower(this.motorPower);
      }
      else {
         testMotor.setPower(0.0);
      }

      updateTestTelemetry(telemetry);
   }

   /**
    * <p>Output test telemetry to driver station</p>
    */
   private void updateTestTelemetry(Telemetry telemetry) {
      testTelemetry.addData("Run Test:", this.runTest);
      testTelemetry.addData("Motor Power:", this.motorPower);
      testTelemetry.addData("Encoder Count: ", testMotor.getCurrentPosition());
   }
}
