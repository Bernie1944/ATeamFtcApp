package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.components.BucketLocalizer;
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.components.DriveLocalizer;
import org.firstinspires.ftc.teamcode.components.Latch;
import org.firstinspires.ftc.teamcode.components.PoseManager;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Vector2;

@TeleOp(name="Tele Op")
public class TeleOpMode extends OpMode {
    // In inches per second
    private static final double MAX_LATCH_TARGET_SPEED = 22.0;

    // In inches per second
    private static final double MAX_NAV_TARGET_SPEED = 22.0;

    // In degrees per second when setting target rotation with joystick
    private static final double MAX_NAV_TARGET_ROTATION_SPEED_FROM_DPAD = 180.0;

    // In degrees when setting target rotation with joystick
    private static final double NAV_TARGET_ROTATION_TOLERANCE_WITH_DPAD = 10.0;

    // In degrees per second when rotating with triggers
    private static final double MAX_NAV_TARGET_ROTATION_SPEED_FROM_TRIGGERS = 100.0;

    // Amount that pressing gamepad 1 bumpers changes current target
    private static final double NAV_ROTATION_TRIM_STEP = 5.0;

    // In inches per second when moving with joystick
    private static final double MAX_BUCKET_TARGET_SPEED_FROM_TRIGGERS = 60.0;

    // In inches per second when moving with joystick
    private static final double MAX_HARVESTER_TARGET_HORIZONTAL_SPEED_FROM_JOYSTICK_WITHOUT_DEPRESSED_TRIGGERS = 20.0;

    // In inches per second when moving with joystick
    private static final double MAX_HARVESTER_TARGET_HORIZONTAL_SPEED_FROM_JOYSTICK_WITH_DEPRESSED_TRIGGERS = 20.0;

    // In inches per second per inch away from ground (0.0)
    private static final double BUCKET_Z_POSITION_CORRECTION_FACTOR = 0.25;

    private static final double MIN_BUCKET_SLACK_OFF_TARGET_FOR_CORRECTION = 0.05;
    private static final double BUCKET_SLACK_GAIN = 0.7;

    // Min target of joystick
    private static final double CONTROLLER_1_JOYSTICK_MAGNITUDE_DEADZONE = 0.14;

    // Min target of joystick
    private static final double CONTROLLER_2_JOYSTICK_MAGNITUDE_DEADZONE = 0.14;

    Latch latch;
    DriveLocalizer driveLocalizer;
    BucketLocalizer bucketLocalizer;
    PoseManager poseManager;
    Controller controller1;
    Controller controller2;

    Pose targetPose;

    double bucketTargetSlack = 1.0;

    @Override
    public void init() {
        latch = new Latch(telemetry, hardwareMap);
        driveLocalizer = new DriveLocalizer(telemetry, hardwareMap, new Vector2(0.0, 0.0), 45.0);
        bucketLocalizer = new BucketLocalizer(telemetry, hardwareMap, driveLocalizer);
        poseManager = new PoseManager(telemetry, hardwareMap, latch, driveLocalizer, bucketLocalizer);
        controller1 = new Controller(telemetry, hardwareMap, gamepad1, CONTROLLER_1_JOYSTICK_MAGNITUDE_DEADZONE);
        controller2 = new Controller(telemetry, hardwareMap, gamepad2, CONTROLLER_2_JOYSTICK_MAGNITUDE_DEADZONE);
    }

    @Override
    public void init_loop() {
        latch.update();
        driveLocalizer.update();
        bucketLocalizer.update();
        poseManager.update();
        controller1.update();
        controller2.update();

        telemetry.addLine(bucketLocalizer.bucket.toString());
        telemetry.addLine(toStringVerbose());

        bucketLocalizer.bucket.setTargetArmRotationVelocity(controller1.getRightJoystickPosition().getY() * 4.0);
    }

    @Override
    public void loop() {
        init_loop();

        // Trim driveLocalizer rotation so robot moves more left
        if (controller2.isLeftBumperPressed()) {
            driveLocalizer.setRotation(driveLocalizer.getRotation() + NAV_ROTATION_TRIM_STEP);
        }

        // Trim driveLocalizer rotation so robot moves more right
        if (controller2.isRightBumperPressed()) {
            driveLocalizer.setRotation(driveLocalizer.getRotation() - NAV_ROTATION_TRIM_STEP);
        }

        // Set driveLocalizer rotation to joystick rotation
        if (controller2.isLeftBumperDown() && controller2.isRightBumperDown() && controller2.getRightJoystickPosition().getMagnitude() == 1.0) {
            driveLocalizer.setRotation(controller2.getRightJoystickPosition().getRotation());
        }

        latch.setTargetVelocity(((controller2.isYButtonDown() ? 1.0 : 0.0) - (controller2.isAButtonDown() ? 1.0 : 0.0)) * MAX_LATCH_TARGET_SPEED);

        if (controller2.isBButtonToggleOn()) {
            driveLocalizer.drive.setTargetVelocityAndTargetRotationVelocity(
                    controller2.getLeftJoystickPosition().mul(MAX_NAV_TARGET_SPEED),
                    (controller2.getLeftTriggerPosition() - controller2.getRightTriggerPosition()) * MAX_NAV_TARGET_ROTATION_SPEED_FROM_TRIGGERS
            );
        } else {
            driveLocalizer.setTargetVelocityAndTargetRotationVelocity(
                    controller2.getLeftJoystickPosition().mul(MAX_NAV_TARGET_SPEED),
                    (controller2.getLeftTriggerPosition() - controller2.getRightTriggerPosition()) * MAX_NAV_TARGET_ROTATION_SPEED_FROM_TRIGGERS
            );
        }

        /*
        double bucketTargetXVelocityFromTriggers = (controller1.getRightTriggerPosition() - controller1.getLeftTriggerPosition()) * MAX_BUCKET_TARGET_SPEED_FROM_TRIGGERS;

        double bucketLocalizerTargetZVelocity = (-bucketLocalizer.getPosition().getZ()) * BUCKET_Z_POSITION_CORRECTION_FACTOR;
        telemetry.addLine("bucketLocalizerTargetZVelocity : " + Inches.toString(bucketLocalizerTargetZVelocity));

        bucketLocalizer.reverseSlideLine.setTargetVelocity(new Vector2(bucketTargetXVelocityFromTriggers, bucketLocalizerTargetZVelocity));

        if (controller1.isLeftBumperDown()) {
            bucketTargetSlack = 1.0;
        } else if (controller1.isRightBumperDown()) {
            bucketTargetSlack = 0.4;
        } else if (controller1.isAButtonDown()) {
            bucketTargetSlack = 0.6;
        } else if (controller1.isBButtonDown()) {
            bucketTargetSlack = 0.0;
        }

        bucketLocalizer.reverseSlideLine.setTargetRotationAmountVelocity((bucketTargetSlack - bucketLocalizer.reverseSlideLine.getRotationAmount()) * BUCKET_SLACK_GAIN);
*/

        Vector2 bucketLocalizerTargetXYVelocityFromJoystick = controller1.getRightJoystickPosition().mul(Range.scale(
                controller1.getLeftTriggerPosition() + controller1.getRightTriggerPosition(),
                0.0, 2.0,
                MAX_HARVESTER_TARGET_HORIZONTAL_SPEED_FROM_JOYSTICK_WITHOUT_DEPRESSED_TRIGGERS, MAX_HARVESTER_TARGET_HORIZONTAL_SPEED_FROM_JOYSTICK_WITH_DEPRESSED_TRIGGERS
        ));

        double bucketTargetXVelocityFromTriggers = (controller1.getRightTriggerPosition() - controller1.getLeftTriggerPosition()) * MAX_BUCKET_TARGET_SPEED_FROM_TRIGGERS;

        double bucketLocalizerTargetZPosition = bucketLocalizer.bucket.getArmPosition() > 0.0 ? 0.0 : 30.0;

        double bucketLocalizerTargetZVelocity = (bucketLocalizerTargetZPosition - bucketLocalizer.getPosition().getZ()) * BUCKET_Z_POSITION_CORRECTION_FACTOR;

        telemetry.addData("bucketLocalizerTargetZVelocity" , bucketLocalizerTargetZVelocity);

        bucketLocalizer.setTargetVelocity(
                bucketLocalizerTargetXYVelocityFromJoystick.add(
                        new Vector2(bucketTargetXVelocityFromTriggers, 0.0).addRotation(driveLocalizer.getRotation())
                ).appendZ(bucketLocalizerTargetZVelocity)
        );

        if (controller1.isRightBumperDown() || controller1.isLeftBumperDown()) {
            bucketLocalizer.bucket.setTargetArmVelocity(
                    (controller1.isRightBumperDown() ? MAX_BUCKET_TARGET_SPEED_FROM_TRIGGERS : 0.0)
                            - (controller1.isLeftBumperDown() ? MAX_BUCKET_TARGET_SPEED_FROM_TRIGGERS : 0.0)
            );
        }

        if (controller1.isLeftBumperDown()) {
            bucketTargetSlack = 1.0;
        } else if (controller1.isRightBumperDown()) {
            bucketTargetSlack = 0.7;
        } else if (controller1.isAButtonDown()) {
            bucketTargetSlack = 0.6;
        } else if (controller1.isBButtonDown()) {
            bucketTargetSlack = 0.0;
        }

        if (Math.abs(bucketTargetSlack - bucketLocalizer.bucket.getSlack()) >= MIN_BUCKET_SLACK_OFF_TARGET_FOR_CORRECTION) {
            bucketLocalizer.bucket.setTargetSlackVelocity((bucketTargetSlack - bucketLocalizer.bucket.getSlack()) * BUCKET_SLACK_GAIN);
        } else {
            bucketLocalizer.bucket.setTargetSlackVelocity(0.0);
        }


        /*
        bucketLocalizer.reverseSlideLine.setTargetVelocityAndTargetRotationAmountVelocity(
                new Vector2(
                        (controller1.getRightTriggerPosition() - controller1.getLeftTriggerPosition()) * 24.0,
                        controller1.getLeftJoystickPosition().getY() * 6.0
                ),
                controller1.getRightJoystickPosition().getY() * 4.0
        );

        Vector2 harvesterTargetHorizontalVelocityFromJoystick = controller1.getRightJoystickPosition().mul(Range.scale(
                controller1.getLeftTriggerPosition() + controller1.getRightTriggerPosition(),
                0.0, 2.0,
                MAX_HARVESTER_TARGET_HORIZONTAL_SPEED_FROM_JOYSTICK_WITHOUT_DEPRESSED_TRIGGERS, MAX_HARVESTER_TARGET_HORIZONTAL_SPEED_FROM_JOYSTICK_WITH_DEPRESSED_TRIGGERS
        ));

        bucketLocalizer.setTargetVelocity(harvesterTargetHorizontalVelocityFromJoystick.appendZ(
                -bucketLocalizer.getPosition().getZ() * BUCKET_Z_POSITION_CORRECTION_FACTOR
        ));

        reverseSlideLine.setTargetVelocity(reverseSlideLine.getTargetVelocity().addX(
                (controller1.getRightTriggerPosition() - controller1.getLeftTriggerPosition()) * MAX_BUCKET_TARGET_SPEED_FROM_TRIGGERS)
        );

        // Calculate robot target target velocity contributed by triggers
        double navTargetRotationVelocityFromTriggers = (controller2.getRightTriggerPosition() - controller2.getLeftTriggerPosition()) * MAX_NAV_TARGET_ROTATION_SPEED_FROM_TRIGGERS;

        // Add driveLocalizer target velocity contributed by triggers
        driveLocalizer.setTargetRotationVelocity(driveLocalizer.getTargetRotationVelocity() + navTargetRotationVelocityFromTriggers);

        if (controller2.getDpadPosition().getMagnitude() == 1.0) {
            poseManager.setTargetNavRotation(new Pose.NavRotation(controller2.getDpadPosition().getRotation(), NAV_TARGET_ROTATION_TOLERANCE_WITH_DPAD));
        }

        if (controller1.isLeftBumperPressed()) {
            if (targetPose == null) {
                targetPose = Pose.PLACING_SLIVER_IN_CARGO_HOLD;
            } else {
                targetPose = null;
            }
        }

        while (targetPose != null && poseManager.isAtPose(targetPose)) {
            targetPose = targetPose.getNext();
        }

        if (targetPose != null) {
            poseManager.setTargetPose(targetPose);
        }
        */
    }

    // Returns text describing state
    @Override
    public String toString() {
        return "latch {\n" +
                latch.toString() + "\n" +
                "}\n" +
                "driveLocalizer {\n" +
                driveLocalizer.toString() + "\n" +
                "}\n" +
                "bucketLocalizer {\n" +
                bucketLocalizer.toString() + "\n" +
                "}\n" +
                "poseManager {\n" +
                poseManager.toString() + "\n" +
                "}";
    }

    // Returns text verbosely describing state
    public String toStringVerbose() {
        return "latch {\n" +
                latch.toStringVerbose() + "\n" +
                "}\n" +
                "driveLocalizer {\n" +
                driveLocalizer.toStringVerbose() + "\n" +
                "}\n" +
                "bucketLocalizer {\n" +
                bucketLocalizer.toStringVerbose() + "\n" +
                "}\n" +
                "poseManager {\n" +
                poseManager.toStringVerbose() + "\n" +
                "}\n" +
                "controller1 {\n" +
                controller1.toStringVerbose() + "\n" +
                "}\n" +
                "controller2 {\n" +
                controller2.toStringVerbose() + "\n" +
                "}";
    }
}