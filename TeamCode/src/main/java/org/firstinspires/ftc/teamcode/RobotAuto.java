package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Robot Auto")
public class RobotAuto extends OpMode {
    MecanumDriveAuto autoDrive = new MecanumDriveAuto();

    private enum Stage {
        PARK,
        READY_TO_START,
        LEAVE_START,
        DRIVING_TO_NEXT_POINT,
        START_STRAFE,
        STRAFING_TO_NEXT_POSITION,
        END_OF_AUTO;
    }


    Stage autoStage = Stage.PARK;

    @Override
    public void init() {
        telemetry.addLine("Begin initializations");

        autoDrive.init(hardwareMap);
        telemetry.addLine("... Mecanum init complete");

        telemetry.addLine("End initializations");

        autoStage = Stage.READY_TO_START;
        telemetry.addLine("READY_TO_START");
    }


    @Override
    public void loop() {
        switch(autoStage) {

            // Initialization completed so LET'S PLAY!!!
            case READY_TO_START:
                autoStage = Stage.LEAVE_START;
                break;

            // Move out of parked position
            case LEAVE_START:
                autoDrive.drive(0.5, 1000);
                autoStage = Stage.DRIVING_TO_NEXT_POINT;
                break;

            // Move out of parked position
            case DRIVING_TO_NEXT_POINT:
                if (autoDrive.isMotionComplete()) {
                    autoStage = Stage.END_OF_AUTO;
                }
                telemetry.addLine("LEAVING_START");
                break;

            // Autonomous is over, leave robot in safe state
            case END_OF_AUTO:
                autoDrive.endOfAutoShutdown();
                telemetry.addLine("END_OF_AUTO");
                break;
        }

        // Update drive station screen
        telemetry.update();
    }
}