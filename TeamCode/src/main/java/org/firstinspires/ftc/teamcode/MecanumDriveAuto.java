package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDriveAuto extends MecanumDrive {

   // 32 counts per inch est.
   public void drive(double power, int distance) {
      stopAndResetEncoders();
      setAllMecanumPowers(power);
      setTargetPosition(distance);
      runToPosition();
   }

   // 39 counts per inch est.
   public void strafe (double power, int distance)
   {
      stopAndResetEncoders();
      setAllMecanumPowers(power);
      setTargetPosition(distance, -distance, distance, -distance);
      runToPosition();
   }

   public void rotate (double power, int distance) {
      stopAndResetEncoders();
      setAllMecanumPowers(power);
      setTargetPosition(distance, -distance, -distance, distance);
      runToPosition();
   }

   public void stopAndResetEncoders() {
      motor_LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motor_RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motor_RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motor_LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
   }

   public void runToPosition() {
      motor_LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      motor_RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      motor_RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      motor_LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
   }

   public void setTargetPosition(int LF_distance, int RF_distance, int RR_distance, int LR_distance)
   {
      motor_LF.setTargetPosition(LF_distance);
      motor_RF.setTargetPosition(RF_distance);
      motor_RR.setTargetPosition(RR_distance);
      motor_LR.setTargetPosition(LR_distance);
   }

   public void setTargetPosition(int distance)
   {
      motor_LF.setTargetPosition(distance);
      motor_RF.setTargetPosition(distance);
      motor_RR.setTargetPosition(distance);
      motor_LR.setTargetPosition(distance);
   }
}
