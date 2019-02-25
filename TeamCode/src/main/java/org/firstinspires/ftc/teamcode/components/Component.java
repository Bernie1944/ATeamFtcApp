package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Provides easy access to several fields that are lost when code is not directly in an op mode
public abstract class Component {
    // Smallest value this class will set deltaTime to and min seconds allowed between calls to updateImpl()
    private static final double MIN_DELTA_TIME = 0.001;

    final Telemetry telemetry;
    final HardwareMap hardwareMap;

    // In seconds of system time at last updateImpl()
    // Initialized to a reasonable value
    double time = (System.nanoTime() / 1000000000.0) - MIN_DELTA_TIME;

    // In seconds between last two calls to updateImpl()
    // Op modes loop about every tenth of a second, but this provides a precise value
    // Initialized to a reasonable value
    double deltaTime = MIN_DELTA_TIME;

    Component(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    // If MIN_DELTA_TIME has past since last updateImpl(), will call updateImpl()
    // This should be called at the beginning of each loop
    public final void update() {
        // There are 1000000000.0 nanoseconds in a second
        double newTime = System.nanoTime() / 1000000000.0;

        // Only call overridable update method updateImpl() if change in time since last update
        // is not so small that division by deltaTime can produce unexpected results such as large numbers, infinity, or NaN
        if (newTime - time >= MIN_DELTA_TIME) {
            deltaTime = newTime - time;
            time = newTime;

            updateImpl();
        }
    }

    // Implementation update to be overridden by inheriting classes
    // Called through update()
    void updateImpl() {}

    // Returns text verbosely describing state
    // If not overridden, simply returns toString()
    public String toStringVerbose() {
        return toString();
    }
}
