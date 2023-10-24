package org.firstinspires.ftc.teamcode.testcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;

public class TestHardware
{
   // Motors
   private DcMotor motor_LF;     // Left front wheel
   private DcMotor motor_RF;     // Right front wheel
   private DcMotor motor_RR;     // Left rear wheel
   private DcMotor motor_LR;     // Right rear wheel
   private DcMotor intakeMotor;  // Pixel intake motor

   public void init (HardwareMap hwMap) {
      motor_LF = hwMap.get(DcMotor.class, "Motor_LF");
      motor_LF.setDirection(DcMotorSimple.Direction.FORWARD);

      motor_RF = hwMap.get(DcMotor.class, "Motor_RF");
      motor_RF.setDirection(DcMotorSimple.Direction.REVERSE);

      motor_RR = hwMap.get(DcMotor.class, "Motor_RR");
      motor_RR.setDirection(DcMotorSimple.Direction.FORWARD);

      motor_LR = hwMap.get(DcMotor.class, "Motor_LR");
      motor_LR.setDirection(DcMotorSimple.Direction.REVERSE);

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
      tests.add(new TestMotor("Drivetrain - Left Front", motor_LF));
      tests.add(new TestMotor("Drivetrain - Right Front", motor_RF));
      tests.add(new TestMotor("Drivetrain - Right Rear", motor_RR));
      tests.add(new TestMotor("Drivetrain - Left Rear", motor_LR));
      tests.add(new TestMotor("Intake Motor", intakeMotor));

      return tests;
   }
}
