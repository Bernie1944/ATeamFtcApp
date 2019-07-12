package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.components.Controller;

@TeleOp(name="Manual Tele Op", group = "Util")
public class ManualTeleOpMode extends OpMode {
    Controller controller1;
    DcMotor bucketLeftSlide;
    DcMotor bucketRightSlide;
    DcMotor bucketPivotAndLatchDrive;
    DcMotor bucketTensioner;

    @Override
    public void init() {
        controller1 = new Controller(telemetry, hardwareMap, "Controller1", gamepad1, 0.0);
        bucketLeftSlide = hardwareMap.dcMotor.get("BucketLeftSlide");
        bucketRightSlide = hardwareMap.dcMotor.get("BucketRightSlide");
        bucketPivotAndLatchDrive = hardwareMap.dcMotor.get("BucketPivotAndLatchDrive");
        bucketTensioner = hardwareMap.dcMotor.get("BucketTensioner");

        // Update telemetry data only before telemetry data is being sent to driver station
        telemetry.addData("State", new Func<String>() {
            @Override
            public String value() {
                return "\n" + ManualTeleOpMode.this.toString();
            }
        });
    }

    @Override
    public void loop() {
        controller1.update();
        bucketLeftSlide.setPower(controller1.getLeftJoystickPosition().getY());
        bucketRightSlide.setPower(-controller1.getLeftJoystickPosition().getY());
        bucketPivotAndLatchDrive.setPower(controller1.getRightJoystickPosition().getY());
        bucketTensioner.setPower(controller1.getRightTriggerPosition() - controller1.getLeftTriggerPosition());
    }

    @Override
    public String toString() {
        return controller1.toString();
    }
}