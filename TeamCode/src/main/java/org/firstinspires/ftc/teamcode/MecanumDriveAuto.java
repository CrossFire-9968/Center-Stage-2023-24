package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDriveAuto extends MecanumDrive
{
   public void drive (double power, int distance) {
      stopAndResetEncoders();
      setMecanumPowers(power);
      setTargetPosition(distance);
      runToPosition();
   }


   public void strafe (double power, int distance)
   {
      stopAndResetEncoders();
      setMecanumPowers(power, -power, power, -power);
      setTargetPosition(distance);
      runToPosition();
   }


   public void rotate (double power, int distance) {
      stopAndResetEncoders();
      setMecanumPowers(power, -power, -power, power);
      setTargetPosition(distance);
      runToPosition();
   }


   // Drive until one of the 4 wheels has reached it's target position. We only wait for one
   // because it is not guaranteed all 4 wheels will reach their target at the same time due
   // to inconsistencies in alignment and resistance in the drivetrain. If we wait for all 4 wheels
   // They will start fighting one another in a tug-of-war type effect and we may not transition
   // to the next stage as we expect.
   public boolean isMotionComplete() {
      boolean motionComplete = false;

      // Only watch until one of the wheels reaches position. At that point, stop all the motors
      if (!motor_LF.isBusy() || !motor_RF.isBusy() || !motor_RR.isBusy() || !motor_LR.isBusy()) {
         stopAndResetEncoders();
         motionComplete = true;
      }

      return (motionComplete);
   }


   private void stopAndResetEncoders() {
      motor_LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motor_RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motor_RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motor_LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
   }


   private void runToPosition() {
      motor_LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      motor_RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      motor_RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      motor_LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
   }


   private void setTargetPosition(int distance)
   {
      motor_LF.setTargetPosition(distance);
      motor_RF.setTargetPosition(distance);
      motor_RR.setTargetPosition(distance);
      motor_LR.setTargetPosition(distance);
   }


   public void endOfAutoShutdown() {
      stopAndResetEncoders();
   }
}
