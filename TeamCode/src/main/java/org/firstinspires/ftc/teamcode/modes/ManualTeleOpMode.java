package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Manual Tele Op")
public class ManualTeleOpMode extends OpMode {
    DcMotor latch;
    DcMotor bucketLift;
    DcMotor bucketReverseSlideLine;
    Servo bucketReclockingServo;
    @Override
    public void init() {
        latch = hardwareMap.dcMotor.get("Latch");
        bucketLift = hardwareMap.dcMotor.get("BucketLift");
        bucketReverseSlideLine = hardwareMap.dcMotor.get("BucketReverseSlideLine");
        bucketReclockingServo = hardwareMap.servo.get("BucketReclockingServo");
    }

    @Override
    public void loop() {
        latch.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        bucketLift.setPower(gamepad1.left_stick_y);
        bucketReverseSlideLine.setPower(gamepad1.right_stick_y);

        if (gamepad1.left_bumper) {
            bucketReclockingServo.setPosition(0.2);
        } else if (gamepad1.right_bumper) {
            bucketReclockingServo.setPosition(0.8);
        }
    }

}