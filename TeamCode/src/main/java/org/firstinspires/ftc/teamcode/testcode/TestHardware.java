package org.firstinspires.ftc.teamcode.testcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;

public class TestHardware
{
   private DcMotor intakeMotor;

   public void init(HardwareMap hwMap)
   {
      intakeMotor = hwMap.get(DcMotor.class, "intake_motor");
   }

   /**
    * <p>Returns list of tests added to the arraylist</p>
    * @param none
    * @return tests ArrayList of all tests available
    */
   public ArrayList<TestItem> getTests()
   {
      ArrayList<TestItem> tests = new ArrayList<>();

      // Add test items to the list here
      tests.add(new TestMotor("Intake Motor", intakeMotor, 1.0));

      return tests;
   }
}
