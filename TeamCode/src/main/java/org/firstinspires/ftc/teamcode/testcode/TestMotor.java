package org.firstinspires.ftc.teamcode.testcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
   This is an test item for testing type DcMotor.
*/
public class TestMotor extends TestItem
{
   private double testSpeed;
   private DcMotor testMotor;

   /**
    * <p>Class used for adding a DcMotor as a testable item</p>
    * @param description Label shown on display
    * @param motor DcMotor object to test
    * @param speed Motor power command for test
    * @return none
    */
   public TestMotor(String description, DcMotor motor, double speed)
   {
      super(description);
      testMotor = motor;
      testSpeed = speed;
   }

   /**
    * <p>Class defining actions to perform during test</p>
    * @param runTest If true, selected test begins
    * @param telemetry Telemetry object to display on driver station during test
    * @return none
    */
   @Override
   public void run(boolean runTest, Telemetry telemetry)
   {
      if (runTest)
      {
         testMotor.setPower(testSpeed);
      }
      else
      {
         testMotor.setPower(0.0);
      }

      // Add DcMotor values of interest here
      telemetry.addData("Motor Power:", testMotor.getPower());

      // If using encoders add to telemetry
      if (testMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER)
      {
         telemetry.addData("Encoder: ", testMotor.getCurrentPosition());
      }
   }
}
