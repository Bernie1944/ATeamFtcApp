package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Degrees;
import org.firstinspires.ftc.teamcode.util.Vector2;

@TeleOp(name="Tele Op")
public class TeleOpMode extends Mode {
    // Amount that pressing controller1 bumpers changes current nav rotation
    private static final double NAV_ROTATION_TRIM_STEP = 9.0;

    // In inches per second
    private static final double MAX_NAV_TARGET_SPEED = 22.0;

    // In degrees per second
    private static final double MAX_NAV_TARGET_ANGULAR_SPEED = 180.0;

    // In degrees per second per degree
    private static final double NAV_ROTATION_GAIN = 2.0;

    // In inches
    private static final double MIN_BUCKET_X_POSITION_FOR_MAX_NAV_ROTATIONAL_MOVEMENT = -6.0;
    private static final double MAX_BUCKET_X_POSITION_FOR_MAX_NAV_ROTATIONAL_MOVEMENT = 8.0;

    // In inches
    private static final double MAX_DRIVE_WHEEL_TARGET_POSITION_OFFSET_WHEN_ADJUSTING_ROTATION = 2.0;
    private static final double BUCKET_SLIDE_TARGET_POSITION_OFFSET_WHEN_ADJUSTING_POSITION = 3.0;

    // In inches per second
    private static final double MAX_DRIVE_WHEEL_SPEED_WHEN_SETTING_DRIVE_WHEEL_TARGET_POSITIONS = 5.0;
    private static final double BUCKET_SLIDE_MAX_SPEED_WHEN_ADJUSTING_POSITION = 10.0;

    // In inches
    private static final double BUCKET_SLIDE_POSITION_TOLERANCE_WHEN_SETTING_TARGET_POSITION = 1.0;
    private static final double DRIVE_WHEEL_POSITION_TOLERANCE_WHEN_SETTING_TARGET_POSITIONS = 1.0;

    private static final double BUCKET_TENSIONER_TARGET_POSITION_FOR_DUMPING_SILVER_MINERALS = 0.67;
    private static final double BUCKET_TENSIONER_TARGET_POSITION_FOR_DUMPING_GOLD_MINERALS = 0.33;
    private static final double BUCKET_TENSIONER_TARGET_POSITION_AT_MIN_SLIDE_POSITION_FOR_SCOOPING_MINERALS = 0.67;
    private static final double BUCKET_TENSIONER_TARGET_POSITION_AT_MAX_SLIDE_POSITION_FOR_SCOOPING_MINERALS = 0.33;

    private double flWheelPositionForDumpingSilverMinerals = 0.0;
    private double frWheelPositionForDumpingSilverMinerals = 0.0;
    private double blWheelPositionForDumpingSilverMinerals = 0.0;
    private double brWheelPositionForDumpingSilverMinerals = 0.0;
    private double bucketSlidePositionForDumpingSilverMinerals = 0.0;

    private double flWheelPositionForDumpingGoldMinerals = 0.0;
    private double frWheelPositionForDumpingGoldMinerals = 0.0;
    private double blWheelPositionForDumpingGoldMinerals = 0.0;
    private double brWheelPositionForDumpingGoldMinerals = 0.0;
    private double bucketSlidePositionForDumpingGoldMinerals = 0.0;

    @Override
    public void update() {
        // Trim nav rotation so robot moves more left
        if (controller1.isLeftBumperPressed()) {
            nav.setRotation(nav.getRotation() - NAV_ROTATION_TRIM_STEP);
        }

        // Trim nav rotation so robot moves more right
        if (controller1.isRightBumperPressed()) {
            nav.setRotation(nav.getRotation() + NAV_ROTATION_TRIM_STEP);
        }

        if (controller2.isXButtonDown()) {
            nav.drive.flWheel.setTargetPosition(flWheelPositionForDumpingSilverMinerals, MAX_DRIVE_WHEEL_SPEED_WHEN_SETTING_DRIVE_WHEEL_TARGET_POSITIONS);
            nav.drive.frWheel.setTargetPosition(frWheelPositionForDumpingSilverMinerals, MAX_DRIVE_WHEEL_SPEED_WHEN_SETTING_DRIVE_WHEEL_TARGET_POSITIONS);
            nav.drive.blWheel.setTargetPosition(blWheelPositionForDumpingSilverMinerals, MAX_DRIVE_WHEEL_SPEED_WHEN_SETTING_DRIVE_WHEEL_TARGET_POSITIONS);
            nav.drive.brWheel.setTargetPosition(brWheelPositionForDumpingSilverMinerals, MAX_DRIVE_WHEEL_SPEED_WHEN_SETTING_DRIVE_WHEEL_TARGET_POSITIONS);
            bucket.slide.setTargetPosition(bucketSlidePositionForDumpingSilverMinerals);
            bucket.correctYPosition();

            if (nav.drive.flWheel.isPositionAt(flWheelPositionForDumpingSilverMinerals, DRIVE_WHEEL_POSITION_TOLERANCE_WHEN_SETTING_TARGET_POSITIONS) &&
                    nav.drive.frWheel.isPositionAt(frWheelPositionForDumpingSilverMinerals, DRIVE_WHEEL_POSITION_TOLERANCE_WHEN_SETTING_TARGET_POSITIONS) &&
                    nav.drive.blWheel.isPositionAt(blWheelPositionForDumpingSilverMinerals, DRIVE_WHEEL_POSITION_TOLERANCE_WHEN_SETTING_TARGET_POSITIONS) &&
                    nav.drive.brWheel.isPositionAt(brWheelPositionForDumpingSilverMinerals, DRIVE_WHEEL_POSITION_TOLERANCE_WHEN_SETTING_TARGET_POSITIONS) &&
                    bucket.slide.isPositionAt(bucketSlidePositionForDumpingSilverMinerals, BUCKET_SLIDE_POSITION_TOLERANCE_WHEN_SETTING_TARGET_POSITION)) {
                bucket.tensioner.setTargetPosition(BUCKET_TENSIONER_TARGET_POSITION_FOR_DUMPING_SILVER_MINERALS);
            } else {
                bucket.tensioner.setTargetPosition(1.0);
            }
        } else if (controller2.isYButtonDown()) {
            nav.drive.flWheel.setTargetPosition(flWheelPositionForDumpingGoldMinerals, MAX_DRIVE_WHEEL_SPEED_WHEN_SETTING_DRIVE_WHEEL_TARGET_POSITIONS);
            nav.drive.frWheel.setTargetPosition(frWheelPositionForDumpingGoldMinerals, MAX_DRIVE_WHEEL_SPEED_WHEN_SETTING_DRIVE_WHEEL_TARGET_POSITIONS);
            nav.drive.blWheel.setTargetPosition(blWheelPositionForDumpingGoldMinerals, MAX_DRIVE_WHEEL_SPEED_WHEN_SETTING_DRIVE_WHEEL_TARGET_POSITIONS);
            nav.drive.brWheel.setTargetPosition(brWheelPositionForDumpingGoldMinerals, MAX_DRIVE_WHEEL_SPEED_WHEN_SETTING_DRIVE_WHEEL_TARGET_POSITIONS);
            bucket.slide.setTargetPosition(bucketSlidePositionForDumpingGoldMinerals);
            bucket.correctYPosition();

            if (nav.drive.flWheel.isPositionAt(flWheelPositionForDumpingGoldMinerals, DRIVE_WHEEL_POSITION_TOLERANCE_WHEN_SETTING_TARGET_POSITIONS) &&
                    nav.drive.frWheel.isPositionAt(frWheelPositionForDumpingGoldMinerals, DRIVE_WHEEL_POSITION_TOLERANCE_WHEN_SETTING_TARGET_POSITIONS) &&
                    nav.drive.blWheel.isPositionAt(blWheelPositionForDumpingGoldMinerals, DRIVE_WHEEL_POSITION_TOLERANCE_WHEN_SETTING_TARGET_POSITIONS) &&
                    nav.drive.brWheel.isPositionAt(brWheelPositionForDumpingGoldMinerals, DRIVE_WHEEL_POSITION_TOLERANCE_WHEN_SETTING_TARGET_POSITIONS) &&
                    bucket.slide.isPositionAt(bucketSlidePositionForDumpingGoldMinerals, BUCKET_SLIDE_POSITION_TOLERANCE_WHEN_SETTING_TARGET_POSITION)) {
                bucket.tensioner.setTargetPosition(BUCKET_TENSIONER_TARGET_POSITION_FOR_DUMPING_GOLD_MINERALS);
            } else {
                bucket.tensioner.setTargetPosition(1.0);
            }
        } else {
            double navTargetAngularVelocityFromController1Triggers =
                    (controller1.getLeftTriggerPosition() - controller1.getRightTriggerPosition()) * MAX_NAV_TARGET_ANGULAR_SPEED;

            double controller1RightJoystickRotationSnappedTo45s = Math.round(controller1.getRightJoystickPosition().getRotation() / 45.0) * 45.0;
            double navTargetAngularVelocityFromController1Joystick = controller1.getRightJoystickPosition().getMagnitude() > 0.5 ?
                    Degrees.normalize(controller1RightJoystickRotationSnappedTo45s - nav.getRotation()) * NAV_ROTATION_GAIN : 0.0;

            double navTargetAngularVelocityFromController2Joystick;
            if (bucket.getXPositionForCorrectYPosition() < MIN_BUCKET_X_POSITION_FOR_MAX_NAV_ROTATIONAL_MOVEMENT) {
                navTargetAngularVelocityFromController2Joystick = -controller2.getLeftJoystickPosition().getX() /
                        (bucket.getXPositionForCorrectYPosition() / MIN_BUCKET_X_POSITION_FOR_MAX_NAV_ROTATIONAL_MOVEMENT) * MAX_NAV_TARGET_ANGULAR_SPEED;
            } else if (bucket.getXPositionForCorrectYPosition() > MAX_BUCKET_X_POSITION_FOR_MAX_NAV_ROTATIONAL_MOVEMENT) {
                navTargetAngularVelocityFromController2Joystick = -controller2.getLeftJoystickPosition().getX() /
                        (bucket.getXPositionForCorrectYPosition() / MAX_BUCKET_X_POSITION_FOR_MAX_NAV_ROTATIONAL_MOVEMENT) * MAX_NAV_TARGET_ANGULAR_SPEED;
            } else {
                navTargetAngularVelocityFromController2Joystick = -controller2.getLeftJoystickPosition().getX() * MAX_NAV_TARGET_ANGULAR_SPEED;
            }

            double navTargetAngularVelocity =
                    navTargetAngularVelocityFromController1Triggers +
                    navTargetAngularVelocityFromController1Joystick +
                    navTargetAngularVelocityFromController2Joystick;

            Vector2 navTargetVelocity = controller1.getLeftJoystickPosition().mul(MAX_NAV_TARGET_SPEED);

            if ((nav.areTargetVelocitiesSet() || !navTargetVelocity.equals(Vector2.ZERO) || navTargetAngularVelocity != 0.0) && !controller2.isLeftButtonDown() && !controller2.isRightButtonDown()) {
                nav.setTargetVelocities(
                        navTargetVelocity,
                        navTargetAngularVelocity
                );
            } else if (controller2.isLeftButtonPressed() || controller2.isRightButtonPressed()) {
                double maxDriveWheelTargetPositionOffset;
                if (controller2.isLeftButtonPressed()) {
                    maxDriveWheelTargetPositionOffset = MAX_DRIVE_WHEEL_TARGET_POSITION_OFFSET_WHEN_ADJUSTING_ROTATION;
                } else if (controller2.isRightButtonPressed()) {
                    maxDriveWheelTargetPositionOffset = -MAX_DRIVE_WHEEL_TARGET_POSITION_OFFSET_WHEN_ADJUSTING_ROTATION;
                } else {
                    maxDriveWheelTargetPositionOffset = 0.0;
                }

                double driveWheelTargetPositionOffset;
                if (bucket.getXPositionForCorrectYPosition() < MIN_BUCKET_X_POSITION_FOR_MAX_NAV_ROTATIONAL_MOVEMENT) {
                    driveWheelTargetPositionOffset = maxDriveWheelTargetPositionOffset /
                            (bucket.getXPositionForCorrectYPosition() / MIN_BUCKET_X_POSITION_FOR_MAX_NAV_ROTATIONAL_MOVEMENT);
                } else if (bucket.getXPositionForCorrectYPosition() > MAX_BUCKET_X_POSITION_FOR_MAX_NAV_ROTATIONAL_MOVEMENT) {
                    driveWheelTargetPositionOffset = maxDriveWheelTargetPositionOffset /
                            (bucket.getXPositionForCorrectYPosition() / MAX_BUCKET_X_POSITION_FOR_MAX_NAV_ROTATIONAL_MOVEMENT);
                } else {
                    driveWheelTargetPositionOffset = maxDriveWheelTargetPositionOffset;
                }

                nav.drive.flWheel.setTargetPosition(
                        nav.drive.flWheel.getTargetPosition() + driveWheelTargetPositionOffset,
                        MAX_DRIVE_WHEEL_SPEED_WHEN_SETTING_DRIVE_WHEEL_TARGET_POSITIONS
                );
                nav.drive.frWheel.setTargetPosition(
                        nav.drive.frWheel.getTargetPosition() + driveWheelTargetPositionOffset,
                        MAX_DRIVE_WHEEL_SPEED_WHEN_SETTING_DRIVE_WHEEL_TARGET_POSITIONS
                );
                nav.drive.blWheel.setTargetPosition(
                        nav.drive.blWheel.getTargetPosition() + driveWheelTargetPositionOffset,
                        MAX_DRIVE_WHEEL_SPEED_WHEN_SETTING_DRIVE_WHEEL_TARGET_POSITIONS
                );
                nav.drive.brWheel.setTargetPosition(
                        nav.drive.brWheel.getTargetPosition() + driveWheelTargetPositionOffset,
                        MAX_DRIVE_WHEEL_SPEED_WHEN_SETTING_DRIVE_WHEEL_TARGET_POSITIONS
                );
            }

            double bucketSlidePower = controller2.getRightTriggerPosition() - controller2.getLeftTriggerPosition();

            if ((bucket.slide.isPowerSet() || bucketSlidePower != 0.0) && !controller2.isDownButtonDown() && !controller2.isUpButtonDown()) {
                bucket.slide.setPower(bucketSlidePower);
            } else if (controller2.isDownButtonPressed() || controller2.isUpButtonPressed()) {
                double bucketSlideTargetPositionOffset;
                if (controller2.isDownButtonPressed()) {
                    bucketSlideTargetPositionOffset = -BUCKET_SLIDE_TARGET_POSITION_OFFSET_WHEN_ADJUSTING_POSITION;
                } else if (controller2.isUpButtonPressed()) {
                    bucketSlideTargetPositionOffset = BUCKET_SLIDE_TARGET_POSITION_OFFSET_WHEN_ADJUSTING_POSITION;
                } else {
                    bucketSlideTargetPositionOffset = 0.0;
                }

                bucket.slide.setTargetPosition(
                        bucket.slide.getTargetPosition() + bucketSlideTargetPositionOffset,
                        BUCKET_SLIDE_MAX_SPEED_WHEN_ADJUSTING_POSITION
                );
            }

            if (controller2.getRightJoystickPosition().getY() != 0.0 || controller2.isRightJoystickToggleOn()) {
                bucket.pivotShaft.setPower(controller2.getRightJoystickPosition().getY());
            } else {
                bucket.correctYPosition();
            }

            if (!controller2.isRightJoystickToggleOn() && bucket.slide.getPosition() > bucket.getMinSlidePositionForScoopingMinerals()) {
                bucket.tensioner.setTargetPosition(Range.scale(bucket.slide.getPosition(),
                        bucket.getMinSlidePositionForScoopingMinerals(), bucket.slide.getMaxPosition(),
                        BUCKET_TENSIONER_TARGET_POSITION_AT_MIN_SLIDE_POSITION_FOR_SCOOPING_MINERALS, BUCKET_TENSIONER_TARGET_POSITION_AT_MAX_SLIDE_POSITION_FOR_SCOOPING_MINERALS
                ));
            } else if (controller2.isAButtonDown()) {
                bucket.tensioner.setTargetPosition(BUCKET_TENSIONER_TARGET_POSITION_FOR_DUMPING_SILVER_MINERALS);

                flWheelPositionForDumpingSilverMinerals = nav.drive.flWheel.getPosition();
                frWheelPositionForDumpingSilverMinerals = nav.drive.frWheel.getPosition();
                blWheelPositionForDumpingSilverMinerals = nav.drive.blWheel.getPosition();
                brWheelPositionForDumpingSilverMinerals = nav.drive.brWheel.getPosition();
                bucketSlidePositionForDumpingSilverMinerals = bucket.slide.getPosition();
            } else if (controller2.isBButtonDown()) {
                bucket.tensioner.setTargetPosition(BUCKET_TENSIONER_TARGET_POSITION_FOR_DUMPING_GOLD_MINERALS);

                flWheelPositionForDumpingGoldMinerals = nav.drive.flWheel.getPosition();
                frWheelPositionForDumpingGoldMinerals = nav.drive.frWheel.getPosition();
                blWheelPositionForDumpingGoldMinerals = nav.drive.blWheel.getPosition();
                brWheelPositionForDumpingGoldMinerals = nav.drive.brWheel.getPosition();
                bucketSlidePositionForDumpingGoldMinerals = bucket.slide.getPosition();
            } else {
                bucket.tensioner.setTargetPosition(1.0);
            }
        }
    }
}