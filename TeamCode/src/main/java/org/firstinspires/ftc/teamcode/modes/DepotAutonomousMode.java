package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.Automator;
import org.firstinspires.ftc.teamcode.components.Bucket;
import org.firstinspires.ftc.teamcode.util.Degrees;
import org.firstinspires.ftc.teamcode.util.Vector2;

@Autonomous(name = "Depot Autonomous")
public class DepotAutonomousMode extends AutonomousMode {
    private static final double INITIAL_NAV_HEADING = -135.0;

    @Override
    double getInitialNavHeading() {
        return INITIAL_NAV_HEADING;
    }

    // In seconds
    private static final double DRIVE_TO_DELIVER_MARKER_TIMEOUT = 7.0;

    private static final double DRIVE_TO_DELIVER_MARKER_NAV_TARGET_HEADING = 135.0;

    private static final double DRIVE_TO_DELIVER_MARKER_NAV_ROTATION_FOR_CLEARING_LATCH = 20.0;

    private static final double DRIVE_TO_DELIVER_MARKER_NAV_TARGET_POSITION_FROM_DIAGONAL = 33.0;

    private static final double DRIVE_TO_DELIVER_MARKER_FINISHED_MAX_NAV_ANGULAR_SPEED = 0.8; //1.0; //3.0;

    // Between [0, 1]
    private static final double DELIVER_MARKER_BUCKET_PIVOT_TARGET_POSITION = 0.528; //0.52; //0.535; //0.53; //0.54; //0.58; //0.59; //0.56; //0.54; //0.533; //0.53; //0.52; //0.54; //0.5; //0.55;

    private void driveToDeliverMarker() throws InterruptedException {
        stepName = "driveToDeliverMarker...";
        stepTimeUntilTimeout = DRIVE_TO_DELIVER_MARKER_TIMEOUT;

        Vector2 navTargetPosition = new Vector2(DRIVE_TO_DELIVER_MARKER_NAV_TARGET_POSITION_FROM_DIAGONAL, 0.0).addRotation(-45.0);

        while (nav.getPosition().getDistanceFrom(navTargetPosition) > NAV_POSITION_TOLERANCE ||
                Degrees.between(nav.getHeading(), DRIVE_TO_DELIVER_MARKER_NAV_TARGET_HEADING) > NAV_HEADING_TOLERANCE ||
                Math.abs(nav.getAngularVelocity()) > DRIVE_TO_DELIVER_MARKER_FINISHED_MAX_NAV_ANGULAR_SPEED) {
            Vector2 navTargetVelocity;
            if (Degrees.between(nav.getHeading(), INITIAL_NAV_HEADING) > DRIVE_TO_DELIVER_MARKER_NAV_ROTATION_FOR_CLEARING_LATCH) {
                navTargetVelocity = navTargetPosition.sub(nav.getPosition()).mul(NAV_POSITION_CORRECTION_GAIN);
                bucket.leftSlide.setTargetPosition(Bucket.MIN_UNEXTENDED_SLIDE_POSITION);
                bucket.rightSlide.setTargetPosition(Bucket.MIN_UNEXTENDED_SLIDE_POSITION);
                bucket.pivot.setTargetPosition(DELIVER_MARKER_BUCKET_PIVOT_TARGET_POSITION);
            } else {
                navTargetVelocity = Vector2.ZERO;
            }

            double navTargetAngularVelocity = Degrees.normalize(DRIVE_TO_DELIVER_MARKER_NAV_TARGET_HEADING - nav.getHeading()) * NAV_HEADING_CORRECTION_GAIN;

            nav.setTargetVelocities(limitNavAcceleration(navTargetVelocity), limitNavAngularAcceleration(navTargetAngularVelocity));

            update();
        }
    }

    // In seconds
    private static final double MOVE_BUCKET_TO_DELIVER_MARKER_TIMEOUT = 3.0;

    // In degrees off target heading for op mode to be terminated
    private static final double MOVE_BUCKET_TO_DELIVER_MARKER_TERMINATION_DISTANCE_OFF_TARGET_NAV_HEADING = 10.0;

    // In inches from min slide position
    private static final double MOVE_BUCKET_TO_DELIVER_MARKER_BUCKET_SLIDE_DISTANCE_FROM_MIN = 7.0; //8.0; //6.0; //4.0; //8.0;

    private void moveBucketToDeliverMarker() throws InterruptedException {
        stepName = "moveBucketToDeliverMarker...";
        stepTimeUntilTimeout = MOVE_BUCKET_TO_DELIVER_MARKER_TIMEOUT;

        nav.setTargetVelocities(Vector2.ZERO, 0.0);
        bucket.leftSlide.setTargetPosition(bucket.leftSlide.getMinPosition() + MOVE_BUCKET_TO_DELIVER_MARKER_BUCKET_SLIDE_DISTANCE_FROM_MIN);
        bucket.rightSlide.setTargetPosition(bucket.leftSlide.getMinPosition() + MOVE_BUCKET_TO_DELIVER_MARKER_BUCKET_SLIDE_DISTANCE_FROM_MIN);
        bucket.pivot.setTargetPosition(DELIVER_MARKER_BUCKET_PIVOT_TARGET_POSITION);

        while (!bucket.leftSlide.isPositionLessThanOrAt(bucket.leftSlide.getMinPosition() + MOVE_BUCKET_TO_DELIVER_MARKER_BUCKET_SLIDE_DISTANCE_FROM_MIN) ||
                bucket.leftSlide.getVelocity() < 0.0) {
            if (Degrees.between(DRIVE_TO_DELIVER_MARKER_NAV_TARGET_HEADING, nav.getHeading()) > MOVE_BUCKET_TO_DELIVER_MARKER_TERMINATION_DISTANCE_OFF_TARGET_NAV_HEADING) {
                throw new InterruptedException();
            }

            update();
        }
    }

    // In seconds
    private static final double OPEN_BUCKET_TO_DELIVER_MARKER_TIMEOUT = 1.5;

    // Between [0, 1]
    private static final double OPEN_BUCKET_TO_DELIVER_MARKER_BUCKET_TENSIONER_POWER = -0.15; //-0.08; //-0.1;

    private static final double OPEN_BUCKET_TO_DELIVER_MARKER_SLIDE_MAX_SPEED = 30.0; //20.0; //8.0;

    private static final double OPEN_BUCKET_TO_DELIVER_MARKER_SLIDE_POWER = -0.65; //-0.6; //-0.75; //-0.8;

    private void openBucketToDeliverMarker() throws InterruptedException {
        stepName = "openBucketToDeliverMarker...";
        stepTimeUntilTimeout = OPEN_BUCKET_TO_DELIVER_MARKER_TIMEOUT;

        nav.setTargetVelocities(Vector2.ZERO, 0.0);
        bucket.pivot.setTargetPosition(DELIVER_MARKER_BUCKET_PIVOT_TARGET_POSITION);
        bucket.tensioner.setPower(OPEN_BUCKET_TO_DELIVER_MARKER_BUCKET_TENSIONER_POWER);
        bucket.leftSlide.setPower(OPEN_BUCKET_TO_DELIVER_MARKER_SLIDE_POWER);
        bucket.rightSlide.setPower(OPEN_BUCKET_TO_DELIVER_MARKER_SLIDE_POWER);

        while (!bucket.leftSlide.isPositionAtMin()) {
            if (Degrees.between(DRIVE_TO_DELIVER_MARKER_NAV_TARGET_HEADING, nav.getHeading()) > MOVE_BUCKET_TO_DELIVER_MARKER_TERMINATION_DISTANCE_OFF_TARGET_NAV_HEADING) {
                throw new InterruptedException();
            }

            update();
        }

        bucket.tensioner.setPower(0.0);
    }

    // In seconds
    private static final double WAIT_WHILE_MARKER_FALLS_DELAY = 0.5;

    private void waitWhileMarkerFalls() throws InterruptedException {
        stepName = "waitWhileMarkerFalls...";
        stepTimeUntilTimeout = Double.POSITIVE_INFINITY;

        nav.setTargetVelocities(Vector2.ZERO, 0.0);
        bucket.pivot.setPower(1.0);
        bucket.tensioner.setPower(0.0);
        bucket.leftSlide.setTargetPosition(bucket.leftSlide.getMinPosition());
        bucket.rightSlide.setTargetPosition(bucket.rightSlide.getMinPosition());

        ElapsedTime elapsedTime = new ElapsedTime();
        while (elapsedTime.seconds() < WAIT_WHILE_MARKER_FALLS_DELAY) {
            update();
        }
    }

    // In seconds
    private static final double UNEXTEND_BUCKET_SLIDE_TIMEOUT = 3.0;

    private static final double UNEXTEND_BUCKET_SLIDE_BUCKET_PIVOT_TARGET_POSITION = 0.7;

    private void unextendBucketSlide() throws InterruptedException {
        stepName = "unextendBucketSlide...";
        stepTimeUntilTimeout = UNEXTEND_BUCKET_SLIDE_TIMEOUT;

        nav.setTargetVelocities(Vector2.ZERO, 0.0);
        bucket.pivot.setTargetPosition(UNEXTEND_BUCKET_SLIDE_BUCKET_PIVOT_TARGET_POSITION);
        bucket.leftSlide.setTargetPosition(bucket.leftSlide.getInitialPosition());
        bucket.rightSlide.setTargetPosition(bucket.rightSlide.getInitialPosition());
        bucket.tensioner.setTargetPosition(bucket.tensioner.getMaxPosition());

        while (bucket.getSlidePosition() < Bucket.MIN_UNEXTENDED_SLIDE_POSITION) {
            if (Degrees.between(DRIVE_TO_DELIVER_MARKER_NAV_TARGET_HEADING, nav.getHeading()) > MOVE_BUCKET_TO_DELIVER_MARKER_TERMINATION_DISTANCE_OFF_TARGET_NAV_HEADING) {
                throw new InterruptedException();
            }

            update();
        }
    }

    // In seconds
    private static final double DRIVE_TO_RECOGNIZE_GOLD_MINERAL_TIMEOUT = 5.0;

    // In inches
    private static final double DRIVE_TO_RECOGNIZE_GOLD_MINERAL_NAV_TARGET_POSITION_FROM_DIAGONAL = 29.0; //26.0; //30.0;

    // In degrees
    private static final double DRIVE_TO_RECOGNIZE_GOLD_MINERAL_NAV_TARGET_HEADING = -153.0; //-160.0; //-164.0; //-163.0; //-162.0; //-153.0; //-150.0; //-160.0;

    private void driveToRecognizeGoldMineral() throws InterruptedException {
        stepName = "driveToRecognizeGoldMineral...";
        stepTimeUntilTimeout = DRIVE_TO_RECOGNIZE_GOLD_MINERAL_TIMEOUT;

        Vector2 navTargetPosition = new Vector2(DRIVE_TO_RECOGNIZE_GOLD_MINERAL_NAV_TARGET_POSITION_FROM_DIAGONAL, 0.0).addRotation(-45.0);

        while (nav.getPosition().getDistanceFrom(navTargetPosition) > NAV_POSITION_TOLERANCE ||
                Degrees.between(nav.getHeading(), DRIVE_TO_RECOGNIZE_GOLD_MINERAL_NAV_TARGET_HEADING) > NAV_HEADING_TOLERANCE) {
            Vector2 navTargetVelocity = navTargetPosition.sub(nav.getPosition()).mul(NAV_POSITION_CORRECTION_GAIN);
            double navTargetAngularVelocity = Degrees.normalize(DRIVE_TO_RECOGNIZE_GOLD_MINERAL_NAV_TARGET_HEADING - nav.getHeading()) * NAV_HEADING_CORRECTION_GAIN;
            nav.setTargetVelocities(limitNavAcceleration(navTargetVelocity), limitNavAngularAcceleration(navTargetAngularVelocity));
            update();
        }
    }

    // In seconds
    private static final double WAIT_FOR_GOLD_MINERAL_TO_BE_RECOGNIZED_GIVE_UP_TIMEOUT = 3.0; //2.0; //1.5;

    // In degrees
    private static final double LEFT_MINERAL_HEADING = -15.0;

    private void waitForGoldMineralToBeRecognized() throws InterruptedException {
        stepName = "waitForGoldMineralToBeRecognized...";
        stepTimeUntilTimeout = Double.POSITIVE_INFINITY;

        nav.setTargetVelocities(Vector2.ZERO, 0.0);

        computerVision.setGoldMineralHeading(LEFT_MINERAL_HEADING);
        computerVision.activateGoldMineralRecognition();

        ElapsedTime elapsedTime = new ElapsedTime();
        while (!computerVision.isGoldMineralRecognized() && elapsedTime.seconds() < WAIT_FOR_GOLD_MINERAL_TO_BE_RECOGNIZED_GIVE_UP_TIMEOUT) {
            update();
        }
    }

    // In seconds
    private static final double REMOVE_GOLD_MINERAL_GIVE_UP_TIMEOUT = 4.0;

    // In inches of farthest away robot will move from the diagonal line stretching between the playing fields two corners and parallel to the row of sampling minerals
    private static final double REMOVE_GOLD_MINERAL_NAV_TARGET_DISTANCE_FROM_DIAGONAL = 48.5; //46.0; //47.5;

    private static final double REMOVE_GOLD_MINERAL_NAV_X_POSITION = 38.0; //36.0; //41.0; //46.0;

    // In inches per second
    private static final double REMOVE_GOLD_MINERAL_MAX_NAV_SPEED = 10.0; //12.0; //14.0; //16.0;

    // In inches per second
    private static final double REMOVE_GOLD_MINERAL_MAX_NAV_ANGULAR_SPEED = 80.0; //100.0; //50.0;

    // In degrees per second per degree
    private static final double REMOVE_GOLD_MINERAL_NAV_HEADING_CORRECTION_GAIN = 1.0;

    private static final double REMOVE_GOLD_MINERAL_MAX_GOLD_MINERAL_HEADING_RELATIVE_TO_CAMERA_FOR_NAV_VELOCITY = 10.0;

    // In inches per second per degree
    private static final double REMOVE_GOLD_MINERAL_NAV_STRAFFING_GAIN = 1.0; //0.3;

    private void removeGoldMineral() throws InterruptedException {
        stepName = "removeGoldMineral...";
        stepTimeUntilTimeout = Double.POSITIVE_INFINITY;

        ElapsedTime elapsedTime = new ElapsedTime();
        while (nav.getPosition().subRotation(-45.0).getX() < REMOVE_GOLD_MINERAL_NAV_TARGET_DISTANCE_FROM_DIAGONAL - NAV_POSITION_TOLERANCE &&
                nav.getPosition().getX() < REMOVE_GOLD_MINERAL_NAV_X_POSITION &&
                (elapsedTime.seconds() < REMOVE_GOLD_MINERAL_GIVE_UP_TIMEOUT || DEBUG_MODE_ENABLED)) {
            nav.setTargetVelocities(
                    limitNavAcceleration(new Vector2(REMOVE_GOLD_MINERAL_MAX_NAV_SPEED, 0.0).addRotation(computerVision.getGoldMineralHeading())),
                    0.0
            );

            update();
        }

        computerVision.deactivateGoldMineralRecognition();

        /*
        stepName = "removeGoldMineral...";
        stepTimeUntilTimeout = Double.POSITIVE_INFINITY;

        ElapsedTime elapsedTime = new ElapsedTime();
        while (nav.getPosition().subRotation(getInitialNavHeading() + 90.0).getX() < REMOVE_GOLD_MINERAL_TARGET_NAV_DISTANCE_FROM_DIAGONAL - NAV_POSITION_TOLERANCE &&
                (elapsedTime.seconds() < REMOVE_GOLD_MINERAL_GIVE_UP_TIMEOUT || DEBUG_MODE_ENABLED)) {
            Vector2 navTargetVelocity;
            if (Math.abs(computerVision.getGoldMineralHeadingRelativeToCamera()) <= REMOVE_GOLD_MINERAL_MAX_GOLD_MINERAL_HEADING_RELATIVE_TO_CAMERA_FOR_NAV_VELOCITY) {
                navTargetVelocity = new Vector2(
                        (REMOVE_GOLD_MINERAL_TARGET_NAV_DISTANCE_FROM_DIAGONAL - nav.getPosition().subRotation(getInitialNavHeading() + 90.0).getX()) *
                                NAV_POSITION_CORRECTION_GAIN,
                        computerVision.getGoldMineralHeadingRelativeToCamera() * REMOVE_GOLD_MINERAL_NAV_STRAFFING_GAIN
                ).addRotation(computerVision.getGoldMineralHeading());

                if (navTargetVelocity.getMagnitude() > REMOVE_GOLD_MINERAL_MAX_NAV_SPEED) {
                    navTargetVelocity = navTargetVelocity.withMagnitude(REMOVE_GOLD_MINERAL_MAX_NAV_SPEED);
                }
            } else {
                navTargetVelocity = Vector2.ZERO;
            }

            double navTargetAngularVelocity = computerVision.getGoldMineralHeadingRelativeToCamera() * REMOVE_GOLD_MINERAL_NAV_HEADING_CORRECTION_GAIN;
            //navTargetAngularVelocity = Range.clip(navTargetAngularVelocity, -REMOVE_GOLD_MINERAL_MAX_NAV_ANGULAR_SPEED, REMOVE_GOLD_MINERAL_MAX_NAV_ANGULAR_SPEED);

            nav.setTargetVelocities(
                    limitNavAcceleration(navTargetVelocity),
                    limitNavAngularAcceleration(navTargetAngularVelocity)
            );

            update();
        }

        computerVision.deactivateGoldMineralRecognition();
        */
    }

    // In seconds
    private static final double DRIVE_AWAY_FROM_MINERALS_TIMEOUT = 3.5;

    // In inches of target distance robot will move from the diagonal line stretching between the playing fields two corners and parallel to the row of sampling minerals
    private static final double DRIVE_AWAY_FROM_MINERALS_TARGET_NAV_DISTANCE_FROM_DIAGONAL = 35.0; //34.5; //33.0; //36.5; //35.5; //34.5; //37.5; //40.5; //41.5; //43.0; //40.5; //38.0;

    private void driveAwayFromMinerals() throws InterruptedException {
        stepName = "driveAwayFromMinerals...";
        stepTimeUntilTimeout = DRIVE_AWAY_FROM_MINERALS_TIMEOUT;

        while (nav.getPosition().subRotation(-45.0).getX() > DRIVE_AWAY_FROM_MINERALS_TARGET_NAV_DISTANCE_FROM_DIAGONAL) {
            double navDistanceFromDiagonal = nav.getPosition().subRotation(-45.0).getX();

            Vector2 navTargetVelocity = new Vector2(
                    (DRIVE_AWAY_FROM_MINERALS_TARGET_NAV_DISTANCE_FROM_DIAGONAL - navDistanceFromDiagonal) * NAV_POSITION_CORRECTION_GAIN,
                    0.0
            ).addRotation(-45.0);

            nav.setTargetVelocities(limitNavAcceleration(navTargetVelocity), 0.0);

            update();
        }
    }

    // In seconds
    private static final double DRIVE_TO_PASS_MINERALS_TIMEOUT = 5.0;

    // In inches from center mineral parallel to row of minerals to nav position for robot to be considered to have passed the minerals
    private static final double DRIVE_TO_PASS_MINERALS_FINISHED_DISTANCE_OFF_CENTER = 29.0; //11.0; //29.0;

    // In degrees
    private static final double DRIVE_TO_PASS_MINERALS_NAV_TARGET_HEADING = 90.0;

    private void driveToPassMinerals() throws InterruptedException {
        stepName = "driveToPassMinerals...";
        stepTimeUntilTimeout = DRIVE_TO_PASS_MINERALS_TIMEOUT;

        while (nav.getPosition().subRotation(-45.0).getY() < DRIVE_TO_PASS_MINERALS_FINISHED_DISTANCE_OFF_CENTER) {
            double navDistanceFromDiagonal = nav.getPosition().subRotation(-45.0).getX();

            Vector2 navTargetVelocity = new Vector2(
                    (DRIVE_AWAY_FROM_MINERALS_TARGET_NAV_DISTANCE_FROM_DIAGONAL - navDistanceFromDiagonal) * NAV_POSITION_CORRECTION_GAIN,
                    MAX_NAV_TARGET_SPEED
            ).addRotation(-45.0);

            double navTargetAngularVelocity = Degrees.normalize(DRIVE_TO_PASS_MINERALS_NAV_TARGET_HEADING - nav.getHeading()) * NAV_HEADING_CORRECTION_GAIN;

            nav.setTargetVelocities(limitNavAcceleration(navTargetVelocity), limitNavAngularAcceleration(navTargetAngularVelocity));

            update();
        }
    }

    // In seconds
    private static final double DRIVE_TO_CRATER_TIMEOUT = 3.5;

    private static final Vector2 DRIVE_TO_CRATER_NAV_TARGET_POSITION = new Vector2(
            55.0,
            6.0 //5.0 //6.5 //3.0 //6.0
    );

    private void driveToCrater() throws InterruptedException {
        stepName = "driveToCrater...";
        stepTimeUntilTimeout = DRIVE_TO_CRATER_TIMEOUT;

        while (nav.getPosition().getDistanceFrom(DRIVE_TO_CRATER_NAV_TARGET_POSITION) > NAV_POSITION_TOLERANCE) {
            Vector2 navTargetVelocity = DRIVE_TO_CRATER_NAV_TARGET_POSITION.sub(nav.getPosition()).mul(NAV_POSITION_CORRECTION_GAIN);

            double navTargetAngularVelocity = Degrees.normalize(DRIVE_TO_PASS_MINERALS_NAV_TARGET_HEADING - nav.getHeading()) * NAV_HEADING_CORRECTION_GAIN;

            nav.setTargetVelocities(limitNavAcceleration(navTargetVelocity), limitNavAngularAcceleration(navTargetAngularVelocity));

            update();
        }
    }

    // In seconds
    private static final double PICKUP_MINERALS_TIMEOUT = 1.5;

    // In inches
    private static final double PICKUP_MINERALS_TERMINATION_BUCKET_SLIDE_MIN_POSITION = 23.0; //21.0; //15.0;

    private void pickupMinerals() throws InterruptedException {
        stepName = "pickupMinerals...";
        stepTimeUntilTimeout = PICKUP_MINERALS_TIMEOUT;

        automator.setMode(Automator.Mode.PICKUP_MINERALS_WITH_DELIVERY);

        while (automator.getMode() == Automator.Mode.PICKUP_MINERALS_WITH_DELIVERY &&
                bucket.getSlidePosition() < PICKUP_MINERALS_TERMINATION_BUCKET_SLIDE_MIN_POSITION) {
            automator.setTargets(Vector2.ZERO, 0.0);
            update();
        }
    }

    @Override
    void runCraterOrDepotOpMode() throws InterruptedException {
        driveToDeliverMarker();
        moveBucketToDeliverMarker();
        openBucketToDeliverMarker();
        waitWhileMarkerFalls();
        unextendBucketSlide();
        driveToRecognizeGoldMineral();
        waitForGoldMineralToBeRecognized();
        removeGoldMineral();
        driveAwayFromMinerals();
        driveToPassMinerals();
        driveToCrater();
        pickupMinerals();
    }
}