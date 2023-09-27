package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RobotControl_Template extends OpMode {
/**
 * <p>
 * This method will be called once, when the INIT button is pressed.
 */
@Override
public void init() {}

/**
 * <p>
 * This method will be called repeatedly during the period between when
 * the init button is pressed and when the play button is pressed (or the
 * OpMode is stopped).
 * <p>
 * This method is optional. By default, this method takes no action.
 */
@Override
public void init_loop() {}

/**
 * <p>
 * This method will be called once, when the play button is pressed.
 * <p>
 * This method is optional. By default, this method takes no action.
 * <p>
 * Example usage: Starting another thread.
 */
@Override
public void start() {}

/**
 * <p>
 * This method will be called repeatedly during the period between when
 * the play button is pressed and when the OpMode is stopped.
 */
@Override
public void loop() {}

/**
 * <p>
 * This method will be called once, when this OpMode is stopped.
 * <p>
 * Your ability to control hardware from this method will be limited.
 * <p>
 * This method is optional. By default, this method takes no action.
 */
@Override
public void stop() {}
}
