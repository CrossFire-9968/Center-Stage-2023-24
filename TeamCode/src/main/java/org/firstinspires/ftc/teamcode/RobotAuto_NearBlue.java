package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot Auto Near Blue")
public class RobotAuto_NearBlue extends LinearOpMode
{
   public DroneLauncher drone = new DroneLauncher();
   public GripperArm gripperArm = new GripperArm();
   public Hang hang = new Hang();
   public Blinkin blinkin = new Blinkin();
   public MecanumDriveAuto mecanumAuto = new MecanumDriveAuto();
   public PixelDetect pixelDetect = new PixelDetect();
   private ElapsedTime cameraTimer = new ElapsedTime();
   private long autoStateDelay = 300;
   private double autoDrivePower = 0.3;

   enum pixelPosition {
      UNKNOWN, LEFT, CENTER, RIGHT
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
      pixelDetect.initTfod(hardwareMap);
      pixelPosition position;
      boolean isAutoComplete = false;

      // Init robot
      mecanumAuto.setAllMecanumPowers(0.0);
      gripperArm.gripperClosed();
      blinkin.setColor(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

      // Wait for the game to start (driver presses PLAY)
      waitForStart();

      // Run this code while Autonomous has not timed out
      while (opModeIsActive() && !isAutoComplete) {
         // drive to position to look for center pixel
         driveToViewPoint();

         // Determine location of pixel
         position = findPixel();

         switch (position)
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

            case UNKNOWN:
            default:
               // what to do if we don't find pixel
               break;
            }

         isAutoComplete = true;
      }
   }


   // Robot drives to position where it attempt to find the pixel before moving to drop on tape
   public void driveToViewPoint() {
      int driveDistanceFromWall = 19;
      int countsToDriveOneInch = -33;

      mecanumAuto.drive(-autoDrivePower, driveDistanceFromWall * countsToDriveOneInch);
      waitForMotionToComplete();
      sleep(autoStateDelay);
   }


   // Method attempts to position robot and use tensorflow to determine location of pixel
   public pixelPosition findPixel() {
      double maxTimeToWait = 2000;                       // milliseconds
      double minConfidence = 0.75;
      int countsToRotateToPixel = 450;                   // 450 is about 45 degrees
      pixelPosition position = pixelPosition.UNKNOWN;

      // Check if pixel is in center location
      // The timer is to allow tensorflow to settle as we have noticed delays in the process
      cameraTimer.reset();
      while ((cameraTimer.milliseconds() < maxTimeToWait) && (position == pixelPosition.UNKNOWN)) {
         if (pixelDetect.getTfodConfidence(telemetry) > minConfidence) {
            position = pixelPosition.CENTER;
            blinkin.setColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);
         }
      }

      // If pixel is not in center position, rotate robot check if pixel is in right location.
      // The timer is to allow tensorflow to settle as we have noticed delays in the process
      if (position == pixelPosition.UNKNOWN) {
         mecanumAuto.rotate(-autoDrivePower, -countsToRotateToPixel);
         waitForMotionToComplete();
         cameraTimer.reset();
         while ((cameraTimer.milliseconds() < maxTimeToWait) && (position == pixelPosition.UNKNOWN)) {
            if (pixelDetect.getTfodConfidence(telemetry) > minConfidence) {
               position = pixelPosition.RIGHT;
               blinkin.setColor(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
         }
      }

      // If pixel is not in right position, assume it is in the left
      // TODO: It rotates the robot back to the look center which should probably be moved to the dropLeftPixel method
      if (position == pixelPosition.UNKNOWN)
      {
         mecanumAuto.rotate(-autoDrivePower, countsToRotateToPixel);
         waitForMotionToComplete();
         position = pixelPosition.LEFT;
         blinkin.setColor(RevBlinkinLedDriver.BlinkinPattern.WHITE);
      }

      return position;
   }


   // Sequence of events for dropping the pixel on the center tape and then parking
   public void dropCenterPixel() {
      int countsToDriveOneInch = -33;        // Approximate encoder counts to drive 1 inch
      int driveDistanceFromWall = 10;        // Inches
      int driveBackwardsToWall = -26;        // Inches
      int rotateToPark = -750;
      int strafeToPark = -50;

      // Drive to tape
      mecanumAuto.drive(-autoDrivePower, driveDistanceFromWall * countsToDriveOneInch);
      waitForMotionToComplete();
      sleep(autoStateDelay);

      // Drop pixel
      gripperArm.gripperOpen();
      sleep(autoStateDelay);

      // Drive backwards after pixel drop
      mecanumAuto.drive(-autoDrivePower, driveBackwardsToWall * countsToDriveOneInch);
      waitForMotionToComplete();
      sleep(autoStateDelay);

      // Strafe to park
      mecanumAuto.strafe(-autoDrivePower, strafeToPark * countsToDriveOneInch);
      waitForMotionToComplete();
      sleep(autoStateDelay);

      // Rotate to orient for teleop
      mecanumAuto.rotate(-autoDrivePower,rotateToPark);
      waitForMotionToComplete();
      sleep(autoStateDelay);
   }



   // Sequence of events for dropping the pixel on the righthand tape and then parking
   public void dropRightPixel() {
      int countsToDriveOneInch = 33;         // Approximate encoder counts to drive 1 inch
      int driveDistanceToTape = -6;          // Inches
      int driveDistanceToDropPixel = 27;     // Inches
      int countsToRotateToPark = -350;       // 450 is about 45 degrees
      int driveDistanceToPark = 22;          // Inches

      // Drive forward to tape
      mecanumAuto.drive(autoDrivePower, driveDistanceToTape * countsToDriveOneInch);
      waitForMotionToComplete();
      sleep(autoStateDelay);

      // Drop pixel
      gripperArm.gripperOpen();
      sleep(autoStateDelay);

      // Drive backwards to lay down pixel
      mecanumAuto.drive(autoDrivePower, driveDistanceToDropPixel * countsToDriveOneInch);
      waitForMotionToComplete();

      // Rotate toward backdrop
      mecanumAuto.rotate(autoDrivePower, countsToRotateToPark);
      waitForMotionToComplete();
      sleep(autoStateDelay);

      // Rotate to orient for teleop
      mecanumAuto.drive(autoDrivePower, driveDistanceToPark * countsToDriveOneInch);
      waitForMotionToComplete();
   }


   // Sequence of events for dropping the pixel on the lefthand tape and then parking
   public void dropLeftPixel() {
      int countsToDriveOneInch = 33;         // Approximate encoder counts to drive 1 inch
      int strafeDistanceToTape = 12;         // Inches
      int driveDistanceToDropPixel = 13;     // Inches
      int countsToRotateToPark = -780;       // 450 is about 45 degrees
      int driveDistanceToPark = 22;          // Inches

      //Strafe
      mecanumAuto.strafe(autoDrivePower, strafeDistanceToTape * countsToDriveOneInch);
      waitForMotionToComplete();
      sleep(autoStateDelay);

      // Drop pixel
      gripperArm.gripperOpen();
      sleep(autoStateDelay);

      // Drive backwards to lay down pixel
      mecanumAuto.drive(autoDrivePower, driveDistanceToDropPixel * countsToDriveOneInch);
      waitForMotionToComplete();
      sleep(autoStateDelay);

      // Rotate toward backdrop
      mecanumAuto.rotate(autoDrivePower, countsToRotateToPark);
      waitForMotionToComplete();
      sleep(autoStateDelay);

      // Drive to park position
      mecanumAuto.drive(autoDrivePower, driveDistanceToPark * countsToDriveOneInch);
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