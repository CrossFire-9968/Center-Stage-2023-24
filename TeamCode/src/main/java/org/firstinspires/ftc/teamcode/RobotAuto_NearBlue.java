package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Robot Auto Near Blue")
public class RobotAuto_NearBlue extends LinearOpMode
{
   public DroneLauncher drone = new DroneLauncher();
   public GripperArm gripperArm = new GripperArm();
   public Hang hang = new Hang();
   public Blinkin blinkin = new Blinkin();
   public MecanumDriveAuto mecanumAuto =  new MecanumDriveAuto();

   enum spikeLocation {
      LEFT, CENTER, RIGHT
   }

   @Override
   public void runOpMode() throws InterruptedException
   {
      //-----------------------------------------------------------------------
      // INITIALIZATIONS
      // Our initialization code should go here before calling "WaitForStart()"
      //-----------------------------------------------------------------------
      drone.init(hardwareMap);
      gripperArm.init(hardwareMap);
      hang.init(hardwareMap);
      blinkin.init(hardwareMap);
      mecanumAuto.init(hardwareMap);
      boolean isAutoComplete = false;

      // Init robot
      mecanumAuto.setAllMecanumPowers(0.0);
      gripperArm.gripperClosed();

      // Until we have camera working, this is a way to cycle pixel location for test
      // Hit a-button to cycle to position
      spikeLocation pixelLocation = spikeLocation.CENTER;

      // Wait for the game to start (driver presses PLAY)
      waitForStart();

      // Run this code while Autonomous has not timed out
      // Run this code while Autonomous has not timed out
      while (opModeIsActive() && !isAutoComplete)
      {
         switch (pixelLocation)
         {
            // Center pixel location detected
            case CENTER:
               dropCenterPixel();
               break;

            // Right pixel location detected
            case RIGHT:
               dropRightPixel();
               break;

            // Left pixel location detected
            case LEFT:
               dropLeftPixel();
               break;
         }

         isAutoComplete = true;
      }
   }

   /**
    * <p> Sequence of events for dropping the pixel on the center tape and then parking </p>
    */
   public void dropCenterPixel() {
      double drivePower = -0.3;                // Motor power
      int countsToDriveOneInch = -33;      // Approximate encoder counts to drive 1 inch
      int driveDistanceFromWall = 29;         // Inches
      int driveDistanceToDropPixel = -28;     // Inches
      int rotateToPark = -750;
      int strafeToPark = -50;

      // Drive forward from wall
      mecanumAuto.drive(drivePower, driveDistanceFromWall * countsToDriveOneInch);
      waitForMotionToComplete();
      sleep(500);

      // Drop pixel
      gripperArm.gripperOpen();
      sleep(500);

      // Drive backwards to lay down pixel
      mecanumAuto.drive(drivePower, driveDistanceToDropPixel * countsToDriveOneInch);
      waitForMotionToComplete();

      // strafe to avoid bars
      mecanumAuto.strafe(drivePower, strafeToPark * countsToDriveOneInch);
      waitForMotionToComplete();
      sleep(500);

      // Rotate towards park position
      mecanumAuto.rotate(drivePower,rotateToPark);
      waitForMotionToComplete();
      sleep(500);
   }

   /**
    * <p> Sequence of events for dropping the pixel on the righthand tape and then parking </p>
    */
   public void dropRightPixel() {
      double drivePower = 0.3;               // Motor power
      int countsToDriveOneInch = 33;      // Approximate encoder counts to drive 1 inch
      int driveDistanceFromWall = -20;        // Inches
      int countsToRotateToPixel = -450;       // 450 is about 45 degrees
      int driveDistanceToTape = -6;           // Inches
      int driveDistanceToDropPixel = 27;    // Inches
      int countsToRotateToPark = -350;      // 450 is about 45 degrees
      int driveDistanceToPark = 22;          // Inches

      // Drive forward from wall
      mecanumAuto.drive(drivePower, driveDistanceFromWall * countsToDriveOneInch);
      waitForMotionToComplete();
      sleep(500);

      // Rotate towards tape
      mecanumAuto.rotate(drivePower, countsToRotateToPixel);
      waitForMotionToComplete();
      sleep(500);

      // Drive forward to tape
      mecanumAuto.drive(drivePower, driveDistanceToTape * countsToDriveOneInch);
      waitForMotionToComplete();
      sleep(500);

      // Drop pixel
      gripperArm.gripperOpen();
      sleep(500);

      // Drive backwards to lay down pixel
      mecanumAuto.drive(drivePower, driveDistanceToDropPixel * countsToDriveOneInch);
      waitForMotionToComplete();

      // Rotate toward backdrop
      mecanumAuto.rotate(drivePower, countsToRotateToPark);
      waitForMotionToComplete();
      sleep(500);

      // Rotate and Drive to park position
      mecanumAuto.drive(drivePower, driveDistanceToPark * countsToDriveOneInch);
      waitForMotionToComplete();
   }

   /**
    * <p> Sequence of events for dropping the pixel on the lefthand tape and then parking </p>
    */
   public void dropLeftPixel(){
      double drivePower = 0.3;               // Motor power
      int countsToDriveOneInch = 33;         // Approximate encoder counts to drive 1 inch
      int driveDistanceFromWall = -20
              ;       // Inches
      int strafeDistanceToTape = 11;         // Inches
      int driveDistanceToDropPixel = 15;     // Inches
      int countsToRotateToPark = -780;       // 450 is about 45 degrees
      int driveDistanceToPark = 22;          // Inches

      // Drive forward from wall
      mecanumAuto.drive(drivePower, driveDistanceFromWall * countsToDriveOneInch);
      waitForMotionToComplete();
      sleep(500);

      //Strafe
      mecanumAuto.strafe(drivePower, strafeDistanceToTape * countsToDriveOneInch);
      waitForMotionToComplete();
      sleep(500);

      // Drop pixel
      gripperArm.gripperOpen();
      sleep(500);

      // Drive backwards to lay down pixel
      mecanumAuto.drive(drivePower, driveDistanceToDropPixel * countsToDriveOneInch);
      waitForMotionToComplete();
      sleep(500);

      // Rotate toward backdrop
      mecanumAuto.rotate(drivePower, countsToRotateToPark);
      waitForMotionToComplete();
      sleep(500);

      // Drive to park position
      mecanumAuto.drive(drivePower, driveDistanceToPark * countsToDriveOneInch);
      waitForMotionToComplete();
   }

   // Drive until one of the 4 wheels has reached it's target position. We only wait for one
   // because it is not guaranteed all 4 wheels will reach their target at the same time due
   // to inconsistencies in alignment and resistance in the drivetrain. If we wait for all 4 wheels
   // They will start fighting one another in a tug-of-war type effect and we may not transition
   // to the next stage as we expect.
   public void waitForMotionToComplete() {

      // Only wait until one of the wheels reaches position. At that point, stop all the motors
      while (mecanumAuto.motor_LF.isBusy() && mecanumAuto.motor_RF.isBusy() && mecanumAuto.motor_RR.isBusy() && mecanumAuto.motor_LR.isBusy()) {
         this.idle();
      }

      mecanumAuto.stopAndResetEncoders();
   }
}