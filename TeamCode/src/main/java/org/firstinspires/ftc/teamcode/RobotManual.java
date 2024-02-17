package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Robot Manual")
public class RobotManual extends OpMode {
    public DroneLauncher drone = new DroneLauncher();
    public GripperArm gripperArm = new GripperArm();
    public Hang hang = new Hang();
    public Blinkin blinkin = new Blinkin();
    public MecanumDrive mecanum =  new MecanumDrive();
    public boolean playWasPressed = false;
    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    @Override
    public void init() {
        drone.init(hardwareMap);
        gripperArm.init(hardwareMap);
        hang.init(hardwareMap);
        blinkin.init(hardwareMap);
        mecanum.init(hardwareMap);

        telemetry.addLine("End of initializations");
        telemetry.update();
    }

    @Override
    public void loop() {
        //Starting timer to change color for endgame.
        if (!playWasPressed) {
            timer.reset();
            playWasPressed = true;
        }

        if (gripperArm.gripperTouchUpper.isPressed()) {
            blinkin.setColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        else if (gripperArm.gripperTouchLower.isPressed()) {
            blinkin.setColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        else if (timer.seconds() >= 90 && timer.seconds() < 120) {
            blinkin.setColor(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        }
        else if (timer.seconds() >= 120) {
            blinkin.setColor(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else {
            blinkin.setColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }

        // The following methods are called iteratively, over and over again
        // Instead of putting all the code in loop(), we break it up into methods
        // to make the code easier to maintain. As we advance in our coding, we'll
        // put these into different classes.
        mecanum.manualDrive(gamepad1);
        drone.control(gamepad2);
        gripperArm.gripperControl(gamepad2);
        int armPosition = gripperArm.armControl(gamepad2);
        gripperArm.armExtension(gamepad2);
        hang.hangerControl(gamepad2);

        telemetry.addData("armPosition: ", armPosition);
    }
}
