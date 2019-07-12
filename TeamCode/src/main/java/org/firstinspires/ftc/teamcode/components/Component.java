package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

// Provides easy access to several fields that are lost when code is not directly in an op mode
public abstract class Component {
    public static class InitializationFailedException extends RuntimeException {
        public InitializationFailedException(String detailMessage) {
            super(detailMessage);
        }
    }

    // Smallest value this class will set deltaTime to and min seconds allowed between calls to internalUpdate()
    private static final double MIN_DELTA_TIME = 0.001;

    final Telemetry telemetry;
    final HardwareMap hardwareMap;
    final String name;

    // In seconds of system time at last internalUpdate()
    // Initialized to a reasonable value
    double time = (System.nanoTime() / 1000000000.0) - MIN_DELTA_TIME;

    // In seconds between last two calls to internalUpdate()
    // Op modes loop about every tenth of a second, but this provides a precise value
    // Initialized to a reasonable value
    double deltaTime = MIN_DELTA_TIME;

    Component(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        name = this.getClass().getSimpleName();
    }

    Component(Telemetry telemetry, HardwareMap hardwareMap, String name) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.name = name;
    }

    // If MIN_DELTA_TIME has past since last internalUpdate(), will call internalUpdate()
    // This should be called at the beginning of each loop
    public final void update() {
        // There are 1000000000.0 nanoseconds in a second
        double newTime = System.nanoTime() / 1000000000.0;

        // Only call overridable update method internalUpdate() if change in time since last update
        // is not so small that division by deltaTime can produce unexpected results such as large numbers, infinity, or NaN
        if (newTime - time >= MIN_DELTA_TIME) {
            deltaTime = newTime - time;
            time = newTime;

            internalUpdate();
        }
    }

    // Implementation update to be overridden by inheriting classes
    // Called through update()
    void internalUpdate() {}

    @Override
    public String toString() {
        return name;
    }

    String createStateString(String caption, Object value) {
        return name + "." + caption + " = " + value.toString() + "\n";
    }

    String createStateString(String caption, String format, Object... args) {
        return name + "." + caption + " = " + String.format(Locale.getDefault(), format, args) + "\n";
    }
}
