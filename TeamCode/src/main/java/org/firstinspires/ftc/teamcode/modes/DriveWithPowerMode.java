package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DriveWithPowerMode")
public class DriveWithPowerMode extends Mode {
    @Override
    public void update() {
        nav.drive.setPowers(controller1.getLeftJoystickPosition(), controller1.getLeftTriggerPosition() - controller1.getRightTriggerPosition());
    }
}