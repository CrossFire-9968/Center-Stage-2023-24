package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot Auto Far Blue")
public class RobotAuto_FarBlue extends LinearOpMode
    {
        public DroneLauncher drone = new DroneLauncher();
        public GripperArm gripperArm = new GripperArm();
        public Hang hang = new Hang();
        public Blinkin blinkin = new Blinkin();
        public MecanumDriveAuto mecanumAuto = new MecanumDriveAuto();
        public PixelDetect pixelDetect = new PixelDetect();
        private ElapsedTime cameraTimer = new ElapsedTime();
        private long autoStateDelay = 300;

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
            org.firstinspires.ftc.teamcode.RobotAuto_NearRed.pixelPosition position;
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
            double drivePower = -0.20;
            int driveDistanceFromWall = 19;
            int countsToDriveOneInch = -33;

            mecanumAuto.drive(drivePower, driveDistanceFromWall * countsToDriveOneInch);
            waitForMotionToComplete();
            sleep(autoStateDelay);
        }


        // Method attempts to position robot and use tensorflow to determine location of pixel
        public org.firstinspires.ftc.teamcode.RobotAuto_NearRed.pixelPosition findPixel() {
            double maxTimeToWait = 3000;                       // milliseconds
            double minConfidence = 0.75;
            double drivePower = -0.20;                         // Motor power
            int countsToRotateToPixel = 450;                   // 450 is about 45 degrees
            org.firstinspires.ftc.teamcode.RobotAuto_NearRed.pixelPosition position = org.firstinspires.ftc.teamcode.RobotAuto_NearRed.pixelPosition.UNKNOWN;

            // Check if pixel is in center location
            // The timer is to allow tensorflow to settle as we have noticed delays in the process
            cameraTimer.reset();
            while ((cameraTimer.milliseconds() < maxTimeToWait) && (position == org.firstinspires.ftc.teamcode.RobotAuto_NearRed.pixelPosition.UNKNOWN)) {
                if (pixelDetect.getTfodConfidence(telemetry) > minConfidence) {
                    position = org.firstinspires.ftc.teamcode.RobotAuto_NearRed.pixelPosition.CENTER;
                    blinkin.setColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                }
            }

            // If pixel is not in center position, rotate robot check if pixel is in left location.
            // The timer is to allow tensorflow to settle as we have noticed delays in the process
            if (position == org.firstinspires.ftc.teamcode.RobotAuto_NearRed.pixelPosition.UNKNOWN) {
                mecanumAuto.rotate(drivePower, countsToRotateToPixel);
                waitForMotionToComplete();
                cameraTimer.reset();
                while ((cameraTimer.milliseconds() < maxTimeToWait) && (position == org.firstinspires.ftc.teamcode.RobotAuto_NearRed.pixelPosition.UNKNOWN)) {
                    if (pixelDetect.getTfodConfidence(telemetry) > minConfidence) {
                        position = org.firstinspires.ftc.teamcode.RobotAuto_NearRed.pixelPosition.LEFT;
                        blinkin.setColor(RevBlinkinLedDriver.BlinkinPattern.RED);
                    }
                }
            }

            // If pixel is not in right position, assume it is in the left
            // TODO: It rotates the robot back to the look center which should probably be moved to the dropLeftPixel method
            if (position == org.firstinspires.ftc.teamcode.RobotAuto_NearRed.pixelPosition.UNKNOWN)
            {
                mecanumAuto.rotate(drivePower, -countsToRotateToPixel);
                waitForMotionToComplete();
                position = org.firstinspires.ftc.teamcode.RobotAuto_NearRed.pixelPosition.RIGHT;
                blinkin.setColor(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            }

            return position;
        }


        // Sequence of events for dropping the pixel on the center tape and then parking
        public void dropCenterPixel() {
            double drivePower = -0.2;              // Motor power
            int countsToDriveOneInch = -33;        // Approximate encoder counts to drive 1 inch
            int driveDistanceToPixel = 11;        // Inches
            int driveForwardsFromPixel = 30;    // Inches
            int rotateToPark = -750;
            int strafeToPark = -122;
            int backwardsToStrafe = -3;
            int strafeToAvoidPixel = 14;
            int driveBackstage = -8;

            // Drive forward from wall.
            mecanumAuto.drive(drivePower, driveDistanceToPixel * countsToDriveOneInch);
            waitForMotionToComplete();
            sleep(autoStateDelay);

            // Drop pixel
            gripperArm.gripperOpen();
            sleep(autoStateDelay);

            // Drive backwards to strafe
            mecanumAuto.drive(drivePower, backwardsToStrafe * countsToDriveOneInch);
            waitForMotionToComplete();
            sleep(autoStateDelay);

            // Strafe to go around pixel
            mecanumAuto.strafe(drivePower, strafeToAvoidPixel * countsToDriveOneInch);
            waitForMotionToComplete();
            sleep(autoStateDelay);

            // Drive forward from pixel
            mecanumAuto.drive(drivePower, driveForwardsFromPixel * countsToDriveOneInch);
            waitForMotionToComplete();
            sleep(autoStateDelay);

            // strafe to avoid bars
            mecanumAuto.strafe(drivePower, strafeToPark * countsToDriveOneInch);
            waitForMotionToComplete();
            sleep(autoStateDelay);

//            // Back into backstage
//            mecanumAuto.drive(drivePower, driveBackstage * countsToDriveOneInch);
//            waitForMotionToComplete();

            // Rotate towards park position
            mecanumAuto.rotate(drivePower,rotateToPark);
            waitForMotionToComplete();
            sleep(autoStateDelay);
        }



        // Sequence of events for dropping the pixel on the right-hand tape and then parking
        public void dropLeftPixel() {
            double drivePower = 0.3;               // Motor power
            int countsToDriveOneInch = 33;         // Approximate encoder counts to drive 1 inch
            int driveDistanceToTape = -5;          // Inches
            int driveDistanceToDropPixel = 5;     // Inches
            int countsToRotateToPark = -450;       // 450 is about 45 degrees
            int driveDistanceToPark = -31;          // Inches
            int strafeToPark = 110;
            int rotateToPark = -750;

            // Drive forward to tape
            mecanumAuto.drive(drivePower, driveDistanceToTape * countsToDriveOneInch);
            waitForMotionToComplete();
            sleep(autoStateDelay);

            // Drop pixel
            gripperArm.gripperOpen();
            sleep(autoStateDelay);

            // Drive backwards to lay down pixel
            mecanumAuto.drive(drivePower, driveDistanceToDropPixel * countsToDriveOneInch);
            waitForMotionToComplete();

            // Rotate toward pixel
            mecanumAuto.rotate(drivePower, countsToRotateToPark);
            waitForMotionToComplete();
            sleep(autoStateDelay);

            // Drive from pixel
            mecanumAuto.drive(drivePower, driveDistanceToPark * countsToDriveOneInch);
            waitForMotionToComplete();

            //strafe to park
            mecanumAuto.strafe(drivePower, strafeToPark * countsToDriveOneInch);
            waitForMotionToComplete();
            sleep(autoStateDelay);

            // Rotate towards park position
            mecanumAuto.rotate(drivePower,rotateToPark);
            waitForMotionToComplete();
            sleep(autoStateDelay);
        }


        // Sequence of events for dropping the pixel on the lefthand tape and then parking
        public void dropRightPixel() {
            double drivePower = 0.3;               // Motor power
            int countsToDriveOneInch = 33;         // Approximate encoder counts to drive 1 inch
            int strafeDistanceToTape = -13;         // Inches
            int driveDistanceToStrafe = -31;     // Inches
            int strafeToPark = 120;
            int rotateToPark = -750;        // Inches
            int rotateCorrect = -20;

            //Strafe
            mecanumAuto.strafe(drivePower, strafeDistanceToTape * countsToDriveOneInch);
            waitForMotionToComplete();
            sleep(autoStateDelay);

            // Drop pixel
            gripperArm.gripperOpen();
            sleep(autoStateDelay);

            // corrective rotation
            mecanumAuto.rotate(drivePower,rotateCorrect);
            waitForMotionToComplete();
            sleep(autoStateDelay);

            // Drive backwards to lay down pixel
            mecanumAuto.drive(drivePower, driveDistanceToStrafe * countsToDriveOneInch);
            waitForMotionToComplete();
            sleep(autoStateDelay);

            //strafe to park
            mecanumAuto.strafe(drivePower, strafeToPark * countsToDriveOneInch);
            waitForMotionToComplete();
            sleep(autoStateDelay);

            // Rotate towards park position
            mecanumAuto.rotate(drivePower,rotateToPark);
            waitForMotionToComplete();
            sleep(autoStateDelay);
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

