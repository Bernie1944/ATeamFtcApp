package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Bucket Demo")
public class BucketDemoMode extends Mode {
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

    double bucketTargetTension = 1.0;

    @Override
    public void update() {
        // Trim nav rotation so robot moves more left
        if (controller2.isLeftBumperPressed()) {
            nav.setRotation(nav.getRotation() + NAV_ROTATION_TRIM_STEP);
        }

        // Trim nav rotation so robot moves more right
        if (controller2.isRightBumperPressed()) {
            nav.setRotation(nav.getRotation() - NAV_ROTATION_TRIM_STEP);
        }

        // Set nav rotation to joystick rotation
        if (controller2.isLeftBumperDown() && controller2.isRightBumperDown() && controller2.getRightJoystickPosition().getMagnitude() == 1.0) {
            nav.setRotation(controller2.getRightJoystickPosition().getRotation());
        }


        if (controller1.isAButtonDown()) bucketTargetTension = 1.0;
        else if (controller1.isBButtonDown()) bucketTargetTension = 0.67;
        else if (controller1.isYButtonDown()) bucketTargetTension = 0.33;
        else if (controller1.isXButtonDown()) bucketTargetTension = 0.0;

        bucket.pivotShaft.setPower(controller1.getLeftJoystickPosition().getY());
        bucket.setPowerAndTargetTension(controller1.getRightJoystickPosition().getY(), bucketTargetTension);


        /*
        latchDrive.setTargetVelocity(((controller2.isYButtonDown() ? 1.0 : 0.0) - (controller2.isAButtonDown() ? 1.0 : 0.0)) * MAX_LATCH_TARGET_SPEED);

        if (controller2.isBButtonToggleOn()) {
            nav.drive.setTargetVelocities(
                    controller2.getLeftJoystickPosition().mul(MAX_NAV_TARGET_SPEED),
                    (controller2.getLeftTriggerPosition() - controller2.getRightTriggerPosition()) * MAX_NAV_TARGET_ROTATION_SPEED_FROM_TRIGGERS
            );
        } else {
            nav.setTargetVelocityAndTargetRotationVelocity(
                    controller2.getLeftJoystickPosition().mul(MAX_NAV_TARGET_SPEED),
                    (controller2.getLeftTriggerPosition() - controller2.getRightTriggerPosition()) * MAX_NAV_TARGET_ROTATION_SPEED_FROM_TRIGGERS
            );
        }

        double bucketTargetXVelocityFromTriggers = (controller1.getRightTriggerPosition() - controller1.getLeftTriggerPosition()) * MAX_BUCKET_TARGET_SPEED_FROM_TRIGGERS;

        double bucketLocalizerTargetZVelocity = (-bucketLocalizer.getSlidePosition().getZ()) * BUCKET_Z_POSITION_CORRECTION_FACTOR;
        telemetry.addLine("bucketLocalizerTargetZVelocity : " + Inches.toString(bucketLocalizerTargetZVelocity));

        bucketLocalizer.reverseSlideLine.setTargetVelocity(new Vector2(bucketTargetXVelocityFromTriggers, bucketLocalizerTargetZVelocity));

        if (controller1.isLeftBumperDown()) {
            bucketTargetTension = 1.0;
        } else if (controller1.isRightBumperDown()) {
            bucketTargetTension = 0.4;
        } else if (controller1.isAButtonDown()) {
            bucketTargetTension = 0.6;
        } else if (controller1.isBButtonDown()) {
            bucketTargetTension = 0.0;
        }

        bucketLocalizer.reverseSlideLine.setTargetRotationAmountVelocity((bucketTargetTension - bucketLocalizer.reverseSlideLine.getRotationAmount()) * BUCKET_SLACK_GAIN);

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
                        new Vector2(bucketTargetXVelocityFromTriggers, 0.0).addRotation(nav.getRotation())
                ).appendZ(bucketLocalizerTargetZVelocity)
        );

        if (controller1.isRightBumperDown() || controller1.isLeftBumperDown()) {
            bucketLocalizer.bucket.setTargetArmVelocity(
                    (controller1.isRightBumperDown() ? MAX_BUCKET_TARGET_SPEED_FROM_TRIGGERS : 0.0)
                            - (controller1.isLeftBumperDown() ? MAX_BUCKET_TARGET_SPEED_FROM_TRIGGERS : 0.0)
            );
        }

        if (bucketLocalizer.bucket.getSlack() > 0.05 && (controller1.isLeftTriggerDown() || controller1.isLeftBumperDown())) {
            bucketLocalizer.bucket.setTargetSlackVelocity(-bucketLocalizer.bucket.getSlack() * BUCKET_SLACK_GAIN);
        } else if (controller1.isAButtonDown()) {
            bucketLocalizer.bucket.setTargetSlackVelocity((0.7 - bucketLocalizer.bucket.getSlack()) * BUCKET_SLACK_GAIN);
        } else if (bucketLocalizer.bucket.getSlack() < 0.95 &&
                (controller1.isBButtonDown() || ((controller1.isRightTriggerDown() || controller1.isRightBumperDown()) && bucketLocalizer.getPosition().getZ() < 1.0))
                ) {
            bucketLocalizer.bucket.setTargetSlackVelocity((1.0 - bucketLocalizer.bucket.getSlack()) * BUCKET_SLACK_GAIN);
        }

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
                -bucketLocalizer.getSlidePosition().getZ() * BUCKET_Z_POSITION_CORRECTION_FACTOR
        ));

        reverseSlideLine.setTargetVelocity(reverseSlideLine.getTargetVelocity().addX(
                (controller1.getRightTriggerPosition() - controller1.getLeftTriggerPosition()) * MAX_BUCKET_TARGET_SPEED_FROM_TRIGGERS)
        );

        // Calculate robot target target velocity contributed by triggers
        double navTargetRotationVelocityFromTriggers = (controller2.getRightTriggerPosition() - controller2.getLeftTriggerPosition()) * MAX_NAV_TARGET_ROTATION_SPEED_FROM_TRIGGERS;

        // Add nav target velocity contributed by triggers
        nav.setTargetRotationVelocity(nav.getTargetAngularVelocity() + navTargetRotationVelocityFromTriggers);

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
}