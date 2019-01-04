package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Controls latch for lowering off of and lifting onto the lander
public class Latch extends Motor {
    private static final String MOTOR_NAME = "Latch";

    private static final double GEAR_RATIO = 3.0;
    private static final double THREADS_PER_INCH = 12.0;

    // In inches from ground to bottom of lift hook
    private static final double INITIAL_POSITION = 16.62;
    private static final double MIN_POSITION = 16.3;
    private static final double MAX_POSITION = 22.5;

    public Latch(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap, MOTOR_NAME, true, GEAR_RATIO / 360.0 / THREADS_PER_INCH, INITIAL_POSITION, MIN_POSITION, MAX_POSITION);
    }
}