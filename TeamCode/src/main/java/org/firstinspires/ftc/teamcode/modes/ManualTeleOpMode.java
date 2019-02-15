package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.Controller;

@TeleOp(name="Manual Tele Op")
public class ManualTeleOpMode extends OpMode {
    Controller controller1;
    DcMotor latchDrive;
    DcMotor bucketPivotShaft;
    DcMotor bucketSlide;
    DcMotor bucketSlideTensioner;

    @Override
    public void init() {
        controller1 = new Controller(telemetry, hardwareMap, gamepad1, 0.14);
        latchDrive = hardwareMap.dcMotor.get("LatchDrive");
        bucketPivotShaft = hardwareMap.dcMotor.get("BucketPivotShaft");
        bucketSlide = hardwareMap.dcMotor.get("BucketSlide");
        bucketSlideTensioner = hardwareMap.dcMotor.get("BucketSlideTensioner");
    }

    @Override
    public void loop() {
        controller1.update();

        latchDrive.setPower((controller1.isRightBumperDown() ? 1.0 : 0.0) - (controller1.isLeftBumperDown() ? 1.0 : 0.0));
        bucketPivotShaft.setPower(controller1.getLeftJoystickPosition().getY());
        bucketSlide.setPower(controller1.getRightJoystickPosition().getY());
        bucketSlideTensioner.setPower(controller1.getRightTriggerPosition() - controller1.getLeftTriggerPosition());
    }

}