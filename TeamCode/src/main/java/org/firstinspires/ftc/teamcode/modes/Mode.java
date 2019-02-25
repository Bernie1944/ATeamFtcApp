package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.Bucket;
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.components.Nav;
import org.firstinspires.ftc.teamcode.util.Vector2;

public abstract class Mode extends OpMode {
    // When using an Xbox 360 controller configured as a Logitech controller under the Driver Station app's Settings,
    // These should be set to a value greater than 0.0 (0.13 - 0.14 seems to work well),
    // which will result in more "smooth" joystick controls that do not seem to "stick" to the x and y axis (this, however, is not preferable in all cases)
    private static final double CONTROLLER_1_JOYSTICK_MAGNITUDE_DEADZONE = 0.0;
    private static final double CONTROLLER_2_JOYSTICK_MAGNITUDE_DEADZONE = 0.14;

    // Smallest value this class will set deltaTime to and min seconds allowed between calls to update()
    private static final double MIN_DELTA_TIME = 0.001;

    private double previousTime = time;

    // In seconds between last two calls to update()
    // Op modes loop about every tenth of a second, but this provides a precise value
    // Initialized to a reasonable value
    double deltaTime = MIN_DELTA_TIME;

    Nav nav;
    Bucket bucket;
    Controller controller1;
    Controller controller2;

    @Override
    public void init() {
        Thread.currentThread().setPriority(Thread.MAX_PRIORITY);

        nav = new Nav(telemetry, hardwareMap);
        bucket = new Bucket(telemetry, hardwareMap);
        controller1 = new Controller(telemetry, hardwareMap, gamepad1, CONTROLLER_1_JOYSTICK_MAGNITUDE_DEADZONE);
        controller2 = new Controller(telemetry, hardwareMap, gamepad2, CONTROLLER_2_JOYSTICK_MAGNITUDE_DEADZONE);
    }

    @Override
    public void init_loop() {
        nav.update();
        bucket.update();
        controller1.update();
        controller2.update();

        telemetry.addLine(toStringVerbose());
    }


    @Override
    public final void loop() {
        init_loop();

        // Only call overridable update method update() if change in time since last update()
        // is not so small that division by deltaTime can produce unexpected results such as large numbers, infinity, or NaN
        if (time - previousTime >= MIN_DELTA_TIME) {
            deltaTime = time - previousTime;
            previousTime = time;

            update();
        }
    }

    public abstract void update();

    // Returns text describing state
    @Override
    public String toString() {
        return "deltaTime : " + String.format("%6.4f", deltaTime) + "\n" +
                "nav {\n" +
                nav.toString() + "\n" +
                "}\n" +
                "bucket {\n" +
                bucket.toString() + "\n" +
                "}";
    }

    // Returns text verbosely describing state
    public String toStringVerbose() {
        return "deltaTime : " + String.format("%6.4f", deltaTime) + "\n" +
                nav.toStringVerbose() + "\n" +
                "}\n" +
                "bucket {\n" +
                bucket.toStringVerbose() + "\n" +
                "}\n" +
                "controller1 {\n" +
                controller1.toString() + "\n" +
                "}\n" +
                "controller2 {\n" +
                controller2.toString() + "\n" +
                "}";
    }
}