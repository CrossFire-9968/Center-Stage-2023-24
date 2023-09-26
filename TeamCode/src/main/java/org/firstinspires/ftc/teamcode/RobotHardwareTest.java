package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.testcode.TestHardware;
import org.firstinspires.ftc.teamcode.testcode.TestItem;

import java.util.ArrayList;

@TeleOp(name = "Robot Hardware Test", group = "Test")
public class RobotHardwareTest extends OpMode
{
   TestHardware testHW = new TestHardware();
   ArrayList<TestItem> tests;
   boolean wasDown;
   boolean wasUp;
   int testIndex;

   @Override
   public void init()
   {
      testHW.init(hardwareMap);
      tests = testHW.getTests();
   }

   /**
    * <p>Allows driver to select a test of choice from the driver station using the robot controller</p>
    *
    * @param none
    * @return none
    */
   @Override
   public void loop()
   {
      TestItem currentTest;

      // Move up through selection each time D-Pad is pressed up
      if (gamepad1.dpad_up && !wasUp)
      {
         testIndex--;
         if (testIndex < 0)
         {
            testIndex = tests.size() - 1;
         }
      }
      wasUp = gamepad1.dpad_up;

      // Move down through selection each time D-Pad is pressed down
      if (gamepad1.dpad_down && !wasDown)
      {
         testIndex++;
         if (testIndex >= tests.size())
         {
            testIndex = 0;
         }
      }
      wasDown = gamepad1.dpad_down;

      // Displays onscreen help for the driver
      telemetry.addLine("Use Up and Down on D-pad to cycle through tests");
      telemetry.addLine("Press A to run test");

      // Runs selected test while gamepad1 a-button is pressed
      currentTest = tests.get(testIndex);
      telemetry.addData("Test:", currentTest.getDescription());
      currentTest.run(gamepad1.a, telemetry);
   }
}
