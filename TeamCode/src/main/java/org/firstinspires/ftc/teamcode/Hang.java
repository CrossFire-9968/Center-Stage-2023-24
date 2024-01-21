package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Hang {
    public DcMotor Hanger_Motor1;
    public DcMotor Hanger_Motor2;
    
    public void init(HardwareMap hwMap){
        Hanger_Motor1 = hwMap.get(DcMotor.class, "Hanger_Motor1");
        Hanger_Motor1.setDirection(DcMotorSimple.Direction.FORWARD);

        Hanger_Motor2 = hwMap.get(DcMotor.class, "Hanger_Motor2");
        Hanger_Motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        Hanger_Motor1.setPower(0.0);
        Hanger_Motor2.setPower(0.0);
}
    public void hangerControl(Gamepad gamepad) {
        // Winch the robot up off the floor
        if (gamepad.dpad_right) {
            Hanger_Motor1.setPower(-1.0);
            Hanger_Motor2.setPower(1.0);
        }

        // Lower the robot toward the floor
        else if (gamepad.dpad_left) {
            Hanger_Motor1.setPower(1.0);
            Hanger_Motor2.setPower(-1.0);
        }
        else {
            Hanger_Motor1.setPower(0.0);
            Hanger_Motor2.setPower(0.0);
        }
    }
}