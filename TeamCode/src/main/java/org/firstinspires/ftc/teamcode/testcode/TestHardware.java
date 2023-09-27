package org.firstinspires.ftc.teamcode.testcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;

public class TestHardware
{
   // Motors
   private DcMotor driveTrainMotor_LF;  // Left front wheel
   private DcMotor driveTrainMotor_RF;  // Right front wheel
   private DcMotor driveTrainMotor_LR;  // Left rear wheel
   private DcMotor driveTrainMotor_RR;  // Right rear wheel
   private DcMotor intakeMotor;         // Pixel intake motor

   public void init (HardwareMap hwMap) {
      driveTrainMotor_LF = hwMap.get(DcMotor.class, "driveMotor_LF");
      driveTrainMotor_LF.setDirection(DcMotorSimple.Direction.REVERSE);

      driveTrainMotor_RF = hwMap.get(DcMotor.class, "driveMotor_RF");
      driveTrainMotor_RF.setDirection(DcMotorSimple.Direction.FORWARD);

      driveTrainMotor_LR = hwMap.get(DcMotor.class, "driveMotor_LR");
      driveTrainMotor_LR.setDirection(DcMotorSimple.Direction.FORWARD);

      driveTrainMotor_RR = hwMap.get(DcMotor.class, "driveMotor_RR");
      driveTrainMotor_RR.setDirection(DcMotorSimple.Direction.REVERSE);

      intakeMotor = hwMap.get(DcMotor.class, "intake_motor");
      intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
}

   /**
    * <p>Returns list of tests added to the arraylist</p>
    * @return tests ArrayList of all tests available
    */
   public ArrayList<TestItem> getTests()
   {
      ArrayList<TestItem> tests = new ArrayList<>();

      // Add test items to the list here
      tests.add(new TestMotor("Drivetrain - Left Front", driveTrainMotor_LF));
      tests.add(new TestMotor("Drivetrain - Right Front", driveTrainMotor_RF));
      tests.add(new TestMotor("Drivetrain - Left Rear", driveTrainMotor_LR));
      tests.add(new TestMotor("Drivetrain - Right Rear", driveTrainMotor_RR));
      tests.add(new TestMotor("Intake Motor", intakeMotor));

      return tests;
   }
}
