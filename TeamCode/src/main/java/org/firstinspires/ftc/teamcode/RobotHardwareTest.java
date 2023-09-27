package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.testcode.TestHardware;
import org.firstinspires.ftc.teamcode.testcode.TestItem;
import java.util.ArrayList;

@TeleOp(name = "Robot Hardware Test", group = "Test")
public class RobotHardwareTest extends OpMode {
   TestHardware testHW = new TestHardware();
   ArrayList<TestItem> tests;
   TestItem currentTest;
   boolean wasDown;
   boolean wasUp;
   int testIndex;

   /**
    * <p>
    * This method will be called once, when the INIT button is pressed.
    */
   @Override
   public void init() {
      testHW.init(hardwareMap);
      tests = testHW.getTests();
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
      // Move up through selection each time D-Pad is pressed up
      if (gamepad1.dpad_up && !wasUp) {
         testIndex--;
         if (testIndex < 0) {
            testIndex = tests.size() - 1;
         }
      }
      wasUp = gamepad1.dpad_up;

      // Move down through selection each time D-Pad is pressed down
      if (gamepad1.dpad_down && !wasDown) {
         testIndex++;
         if (testIndex >= tests.size()) {
            testIndex = 0;
         }
      }
      wasDown = gamepad1.dpad_down;

      // Displays onscreen help for the driver
      telemetry.addLine("Use D-pad ↑↓ to select test");
      telemetry.addLine("Hold A to run test");
      telemetry.addLine("  - For motor: use left joystick ↑↓");
      telemetry.addLine("----------------------------------");


      // Runs selected test while gamepad1 a-button is pressed
      currentTest = tests.get(testIndex);
      telemetry.addData("Test:", currentTest.getDescription());
      currentTest.run(gamepad1.a, telemetry, gamepad1);
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
}
