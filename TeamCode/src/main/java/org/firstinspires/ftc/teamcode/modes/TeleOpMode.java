package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.components.Bucket;
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.components.Nav;
import org.firstinspires.ftc.teamcode.components.Latch;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Vector2;

@TeleOp(name="Tele Op")
public class TeleOpMode extends Mode {
    // Amount that pressing gamepad 1 bumpers changes current target
    private static final double NAV_ROTATION_TRIM_STEP = 5.0;

    // In inches per second
    private static final double MAX_NAV_TARGET_SPEED = 22.0;

    // In degrees per second when setting target rotation with joystick
    private static final double MAX_NAV_TARGET_ANGULAR_SPEED_FROM_TRIGGERS = 180.0;

    private static final double NAV_ANGULAR_VELOCITY_FROM_JOYSTICK_GAIN = 2.0;

    // In degrees per second when rotating with triggers
    private static final double MIN_BUCKET_SLIDE_POSITION_MAGNITUDE_FOR_DECREASED_DRIVE_ANGULAR_POWER = 8.0;

    @Override
    public void update() {
        // Trim nav rotation so robot moves more left
        if (controller1.isLeftBumperPressed()) {
            nav.setRotation(nav.getRotation() + NAV_ROTATION_TRIM_STEP);
        }

        // Trim nav rotation so robot moves more right
        if (controller1.isRightBumperPressed()) {
            nav.setRotation(nav.getRotation() - NAV_ROTATION_TRIM_STEP);
        }

        double navTargetAngularVelocityFromTriggers =
                (controller1.getLeftTriggerPosition() - controller1.getRightTriggerPosition() - controller2.getLeftJoystickPosition().getX()) *
                MAX_NAV_TARGET_ANGULAR_SPEED_FROM_TRIGGERS;

        double navTargetAngularVelocityFromJoystick = (controller1.getRightJoystickPosition().getMagnitude() > 0.5) ?
            (controller1.getRightJoystickPosition().getRotation() - nav.getRotation()) * NAV_ANGULAR_VELOCITY_FROM_JOYSTICK_GAIN :
                0.0;

        nav.setTargetVelocities(
                controller1.getLeftJoystickPosition().mul(MAX_NAV_TARGET_SPEED),
                navTargetAngularVelocityFromTriggers + navTargetAngularVelocityFromJoystick
        );

        bucket.slide.setPower(controller2.getRightTriggerPosition() - controller2.getLeftTriggerPosition());

        if (controller2.isYButtonToggleOn()) {
            bucket.pivotShaft.setPower(controller2.getRightJoystickPosition().getY());
        } else {
            bucket.setTargetYPosition();
        }

        if (!controller2.isYButtonToggleOn() && bucket.slide.getPosition() > 16.0) {
            bucket.tensioner.setTargetPosition(Range.scale(bucket.slide.getPosition(), 16.0, bucket.slide.getMaxPosition(), 0.67, 0.33));
        } else if (controller2.isAButtonDown()) {
            bucket.tensioner.setTargetPosition(0.67);
        } else if (controller2.isBButtonDown()) {
            bucket.tensioner.setTargetPosition(0.33);
        } else {
            bucket.tensioner.setTargetPosition(1.0);
        }

        /*
        if (bucket.slide.getPosition() < MIN_BUCKET_SLIDE_POSITION_MAGNITUDE_FOR_DECREASED_DRIVE_ANGULAR_POWER) {
            nav.drive.setPowers(controller1.getLeftJoystickPosition(), controller1.getLeftTriggerPosition() - controller1.getRightTriggerPosition());
        } else {
            nav.drive.setPowers(
                    controller1.getLeftJoystickPosition(),
                    (controller1.getLeftTriggerPosition() - controller1.getRightTriggerPosition()) /
                            Math.abs(bucket.slide.getPosition() / MIN_BUCKET_SLIDE_POSITION_MAGNITUDE_FOR_DECREASED_DRIVE_ANGULAR_POWER));
        }

        // Set nav rotation to joystick rotation
        if (controller2.isLeftBumperDown() && controller2.isRightBumperDown() && controller2.getRightJoystickPosition().getMagnitude() == 1.0) {
            nav.setRotation(controller2.getRightJoystickPosition().getRotation());
        }

        if (controller1.isLeftBumperDown()) {
            nav.setTargetVelocities(
                    controller1.getLeftJoystickPosition().mul(MAX_NAV_TARGET_SPEED),
                    (controller1.getRightTriggerPosition() - controller1.getLeftTriggerPosition()) * MAX_NAV_TARGET_ROTATION_SPEED_FROM_TRIGGERS
            );
        } else {
            nav.setTargetOrientation(
                    controller1.getRightJoystickPosition().mul(24.0),
                    controller1.getDpadPosition().getRotation()
            );
        }
*/

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
            bucketTensionerTargetPosition = 1.0;
        } else if (controller1.isRightBumperDown()) {
            bucketTensionerTargetPosition = 0.4;
        } else if (controller1.isAButtonDown()) {
            bucketTensionerTargetPosition = 0.6;
        } else if (controller1.isBButtonDown()) {
            bucketTensionerTargetPosition = 0.0;
        }

        bucketLocalizer.reverseSlideLine.setTargetRotationAmountVelocity((bucketTensionerTargetPosition - bucketLocalizer.reverseSlideLine.getRotationAmount()) * BUCKET_SLACK_GAIN);

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