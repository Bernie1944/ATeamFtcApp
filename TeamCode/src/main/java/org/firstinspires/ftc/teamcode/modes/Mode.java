package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.components.Bucket;
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.components.Nav;
import org.firstinspires.ftc.teamcode.components.Latch;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Vector2;

public abstract class Mode extends OpMode {
    // When using an Xbox 360 controller configured as a Logitech controller under the Driver Station app's Settings,
    // These should be set to a value greater than 0.0 (0.13 - 0.14 seems to work well),
    // which will result in more "smooth" joystick controls that do not seem to "stick" to the x and y axis (this, however, is not preferable in all cases)
    private static final double CONTROLLER_1_JOYSTICK_MAGNITUDE_DEADZONE = 0.14;
    private static final double CONTROLLER_2_JOYSTICK_MAGNITUDE_DEADZONE = 0.14;

    Latch latch;
    Nav nav;
    Bucket bucket;
    Controller controller1;
    Controller controller2;

    Pose targetPose;

    @Override
    public void init() {
        latch = new Latch(telemetry, hardwareMap);
        nav = new Nav(telemetry, hardwareMap, new Vector2(0.0, 0.0), 0.0);
        bucket = new Bucket(telemetry, hardwareMap);
        controller1 = new Controller(telemetry, hardwareMap, gamepad1, CONTROLLER_1_JOYSTICK_MAGNITUDE_DEADZONE);
        controller2 = new Controller(telemetry, hardwareMap, gamepad2, CONTROLLER_2_JOYSTICK_MAGNITUDE_DEADZONE);
    }

    @Override
    public void init_loop() {
        latch.update();
        nav.update();
        bucket.update();
        controller1.update();
        controller2.update();

        telemetry.addLine(toStringVerbose());
    }


    @Override
    public final void loop() {
        init_loop();

        update();
    }

    public abstract void update();

    // Returns text describing state
    @Override
    public String toString() {
        return "latchDrive {\n" +
                latch.toString() + "\n" +
                "}\n" +
                "nav {\n" +
                nav.toString() + "\n" +
                "}\n" +
                "bucket {\n" +
                bucket.toString() + "\n" +
                "}";
    }

    // Returns text verbosely describing state
    public String toStringVerbose() {
        return "latchDrive {\n" +
                latch.toStringVerbose() + "\n" +
                "}\n" +
                "nav {\n" +
                nav.toStringVerbose() + "\n" +
                "}\n" +
                "bucket {\n" +
                bucket.toStringVerbose() + "\n" +
                "}\n" +
                "controller1 {\n" +
                controller1.toStringVerbose() + "\n" +
                "}\n" +
                "controller2 {\n" +
                controller2.toStringVerbose() + "\n" +
                "}";
    }
}