package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;

import org.firstinspires.ftc.teamcode.components.Automator;
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.components.Nav;
import org.firstinspires.ftc.teamcode.components.ComputerVision;
import org.firstinspires.ftc.teamcode.components.Bucket;
import org.firstinspires.ftc.teamcode.components.Latch;
import org.firstinspires.ftc.teamcode.util.Vector2;

import java.util.Locale;

public abstract class AutonomousMode extends LinearOpMode {
    // Should stepTimeUntilTimeout be ignored and should the controller 1 A button toggle freezing the robot?
    static final boolean DEBUG_MODE_ENABLED = false;

    // In inches per second per inch
    static final double NAV_POSITION_CORRECTION_GAIN = 1.75;

    // In degrees per second per degree
    static final double NAV_HEADING_CORRECTION_GAIN = 2.2; //2.8; //2.9;

    // In inches
    static final double NAV_POSITION_TOLERANCE = 1.0; //0.75 //0.5;

    // In degrees
    static final double NAV_HEADING_TOLERANCE = 3.0; //2.0;

    // In inches per second
    static final double MAX_NAV_TARGET_SPEED = 22.0;

    // In degrees per second
    static final double MAX_NAV_TARGET_ANGULAR_SPEED = 180.0;

    // In inches per second per second
    private static final double MAX_NAV_ACCELERATION = 150.0; //50.0; //100.0; //48.0; //24.0; //50.0; //150.0;

    // In degrees per second per second
    private static final double MAX_NAV_ANGULAR_ACCELERATION = 2000.0; //500.0; //360.0; //800.0; //500.0; //2000.0;

    // time at previous call to update()
    private double previousUpdateTime;

    // In seconds between last two calls to update()
    // Op modes loop about every tenth of a second, but this provides a precise value
    // Initialized to a reasonable value
    double deltaTime;

    String stepName = "...";

    // Seconds left until update() will thrown an InterruptedException
    double stepTimeUntilTimeout = Double.POSITIVE_INFINITY;

    Nav nav;
    ComputerVision computerVision;
    Bucket bucket;
    Latch latch;
    Automator automator;

    Controller controller1;

    Vector2 limitNavAcceleration(Vector2 targetVelocity) {
        double requestedAcceleration = targetVelocity.sub(nav.getVelocity()).getMagnitude() / deltaTime;

        // Limit requestedAccelerationMagnitude to MAX_NAV_ACCELERATION
        if (requestedAcceleration > MAX_NAV_ACCELERATION) {
            // Using requestedAccelerationMagnitude, fade targetVelocity between targetVelocity and current velocity
            return targetVelocity.mul(MAX_NAV_ACCELERATION / requestedAcceleration)
                    .add(nav.getVelocity().mul(1.0 - (MAX_NAV_ACCELERATION / requestedAcceleration)));
        } else {
            return targetVelocity;
        }
    }

    double limitNavAngularAcceleration(double targetAngularVelocity) {
        double requestedAngularAcceleration = Math.abs(targetAngularVelocity - nav.getAngularVelocity()) / deltaTime;

        // Limit requestedAngularAcceleration to MAX_NAV_ANGULAR_ACCELERATION
        if (requestedAngularAcceleration > MAX_NAV_ANGULAR_ACCELERATION) {
            // Using requestedAngularAcceleration, fade targetAngularVelocity between targetAngularVelocity and current angular velocity
            return (targetAngularVelocity * (MAX_NAV_ANGULAR_ACCELERATION / requestedAngularAcceleration)) +
                    (nav.getAngularVelocity() * (1.0 - (MAX_NAV_ANGULAR_ACCELERATION / requestedAngularAcceleration)));
        } else {
            return targetAngularVelocity;
        }
    }

    private void stopRobot() {
        nav.drive.flWheel.brake();
        nav.drive.frWheel.brake();
        nav.drive.blWheel.brake();
        nav.drive.brWheel.brake();
        bucket.leftSlide.brake();
        bucket.rightSlide.brake();
        bucket.pivot.brake();
        bucket.tensioner.brake();
    }

    private static class TimeoutException extends RuntimeException {}

    // Update component status (hardware is updated automatically)
    void update() throws InterruptedException, TimeoutException {
        do {
            if (isStopRequested()) {
                stopRobot();
                throw new InterruptedException();
            }

            // Here update() is only allowed to run once per hardware cycle; otherwise, certain logic will not perform correctly
            // For example, a Motor might report its velocity as zero because its position reportedly had not changed since last update while in reality it is moving
            // time is updated each hardware cycle, so time is compared to previousUpdateTime to see if a hardware cycle has passed
            while (time == previousUpdateTime) {
                sleep(5);
                if (isStopRequested()) {
                    stopRobot();
                    throw new InterruptedException();
                }
            }

            deltaTime = time - previousUpdateTime;
            previousUpdateTime = time;

            nav.update();
            computerVision.update();
            bucket.update();
            latch.update();
            if (DEBUG_MODE_ENABLED) controller1.update();

            if (!DEBUG_MODE_ENABLED || !controller1.isAButtonToggleOn()) {
                stepTimeUntilTimeout -= deltaTime;
                telemetry.update();
            }

            if (!DEBUG_MODE_ENABLED && stepTimeUntilTimeout <= 0.0) {
                stopRobot();
                throw new TimeoutException();
            }

            if (DEBUG_MODE_ENABLED && controller1.isAButtonToggleActivated()) {
                // Freeze robot and telemetry
                nav.drive.flWheel.setTargetPosition(nav.drive.flWheel.getPosition());
                nav.drive.frWheel.setTargetPosition(nav.drive.frWheel.getPosition());
                nav.drive.blWheel.setTargetPosition(nav.drive.blWheel.getPosition());
                nav.drive.brWheel.setTargetPosition(nav.drive.brWheel.getPosition());
                bucket.leftSlide.setTargetPosition(bucket.leftSlide.getPosition());
                bucket.rightSlide.setTargetPosition(bucket.rightSlide.getPosition());
                bucket.pivot.setTargetPosition(bucket.pivot.getPosition());
                bucket.tensioner.setTargetPosition(bucket.tensioner.getPosition());
            }

            // Check again in case isStopRequested() has changed for fastest interrupt
            if (isStopRequested()) {
                stopRobot();
                throw new InterruptedException();
            }
        } while (DEBUG_MODE_ENABLED && controller1.isAButtonToggleOn());
    }

    // In inches from center of lander
    private static final double INITIAL_NAV_POSITION_MAGNITUDE = 21.4;

    abstract double getInitialNavHeading();

    // In degrees of assumed gold mineral heading relative to camera when at initial nav heading
    private static final double INITIAL_GOLD_MINERAL_HEADING_RELATIVE_TO_CAMERA = 25.0;

    // Between [0, 1]
    private static final double BUCKET_TENSIONER_POSITION_FOR_MARKER = 0.75;

    private void initAndWaitForStart() throws InterruptedException {
        stepName = "initAndWaitForStart...";
        stepTimeUntilTimeout = Double.POSITIVE_INFINITY;

        previousUpdateTime = time;

        nav = new Nav(telemetry, hardwareMap);
        computerVision = new ComputerVision(telemetry, hardwareMap, nav);
        latch = new Latch(telemetry, hardwareMap, true);
        bucket = new Bucket(telemetry, hardwareMap, nav, latch);
        automator = new Automator(telemetry, hardwareMap, nav, bucket, latch, getInitialNavHeading());
        if (DEBUG_MODE_ENABLED) controller1 = new Controller(telemetry, hardwareMap, "Controller1", gamepad1, 0.0);

        // Update telemetry data only before telemetry data is being sent to driver station
        telemetry.addData("State", new Func<String>() {
            @Override
            public String value() {
                return "\n" + AutonomousMode.this.toString();
            }
        });

        do {
            nav.setPosition(new Vector2(INITIAL_NAV_POSITION_MAGNITUDE, 0.0).addRotation(getInitialNavHeading() + 90.0));
            nav.setHeading(getInitialNavHeading());
            update();
        } while (!isStarted());

        bucket.tensioner.setTargetPosition(BUCKET_TENSIONER_POSITION_FOR_MARKER);
    }

    // In seconds
    private static final double DISENGAGE_LATCH_TIMEOUT = 4.0;

    private void disengageLatch() throws InterruptedException {
        stepName = "disengageLatch...";
        stepTimeUntilTimeout = DISENGAGE_LATCH_TIMEOUT;

        while (!latch.isDisengaged()) {
            latch.disengage();
            update();
        }
    }

    abstract void runCraterOrDepotOpMode() throws InterruptedException;

    @Override
    public final void runOpMode() {
        try {
            initAndWaitForStart();
            disengageLatch();
            runCraterOrDepotOpMode();

            stepName = "Autonomous completed.";
        } catch (InterruptedException interruptedException) {
            Thread.currentThread().interrupt();

            stepName = stepName + " interrupted!";
        } catch (TimeoutException timeoutException) {
            stepName = stepName + " timed out!";
        } finally {
            stopRobot();
        }
    }

    @Override
    public String toString() {
        return "AutonomousMode.stepName = " + stepName + "\n" +
                "AutonomousMode.stepTimeUntilTimeout = " + String.format(Locale.getDefault(), "%.1f", stepTimeUntilTimeout) + "\n" +
                nav.toString() + computerVision.toString() + bucket.toString() + latch.toString();
    }
}