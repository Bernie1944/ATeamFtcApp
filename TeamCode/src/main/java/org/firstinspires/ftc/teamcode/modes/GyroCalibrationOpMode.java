package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.BucketLocalizer;
import org.firstinspires.ftc.teamcode.components.DriveLocalizer;
import org.firstinspires.ftc.teamcode.components.Gyro;
import org.firstinspires.ftc.teamcode.components.Latch;
import org.firstinspires.ftc.teamcode.components.PoseManager;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Vector2;

@Autonomous(name="Gyro Calibration")
public class GyroCalibrationOpMode extends OpMode {
    private static final String GYRO_NAME = "DriveLocalizerGyro";

    Gyro gyro;

    @Override
    public void init() {
        gyro = new Gyro(telemetry, hardwareMap, GYRO_NAME, 0.0);
    }

    @Override
    public void start() {
        gyro.calibrate();
        requestOpModeStop();
    }

    @Override
    public void loop() {}
}