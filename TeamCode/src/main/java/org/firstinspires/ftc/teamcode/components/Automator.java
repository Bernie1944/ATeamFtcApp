package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Degrees;
import org.firstinspires.ftc.teamcode.util.Vector2;

public class Automator extends Component {
    // In degrees per second per degree
    private static final double NAV_HEADING_CORRECTION_GAIN = 4.0; //2.9;

    // Between [0, 1]
    private static final double COMPLETELY_OPEN_BUCKET_TENSIONER_POWER = -0.15; //-0.11; //-0.08; //-0.1;

    // In inches per second
    private static final double MINERAL_PICKUP_WITH_DELIVERY_BUCKET_EXTENDED_FORWARD_SLIDE_POWER = 0.6;

    // In inches per second for detecting that the bucket has picked up minerals
    private static final double MINERAL_PICKUP_WITH_DELIVERY_FINISHED_MAX_BUCKET_SLIDE_VELOCITY = 12.0;

    // Between [0, 1] of min bucket tensioner position for bucket to be consider to be closed and for slide to move towards back of robot
    private static final double MINERAL_DELIVERY_MIN_CLOSED_BUCKET_TENSIONER_POSITION = 0.93; //0.8;

    // In negative inches per second
    private static final double DELIVER_MINERALS_MIN_BUCKET_SLIDE_VELOCITY_FOR_ACCIDENTALLY_JERKING_MINERALS_OUT = -12.0;

    // In negative inches per second
    private static final double DELIVER_MINERALS_MAX_BUCKET_TENSIONER_POSITION_FOR_ACCIDENTALLY_JERKING_MINERALS_OUT = 0.5;

    // Between [0, 1] for dropping silver minerals without dropping gold minerals
    private static final double SILVER_ONLY_DELIVERY_BUCKET_TENSIONER_TARGET_POSITION = 0.18; //0.21; //0.25; //0.2; //0.3; //0.44; //0.43; //0.42; //0.45; //0.5; //0.4; //0.42; //0.5; //0.4; //0.3;

    // In tensioner travel per second for dropping silver minerals without dropping gold minerals
    private static final double SILVER_ONLY_DELIVERY_BUCKET_TENSIONER_MAX_VELOCITY = 1.3; //1.2; //1.5; //2.0; //1.0;

    // In tensioner travel per second
    private static final double BUCKET_TENSIONER_MAX_VELOCITY = 5.0; //3.5; //2.2;

    // In inches relative to center of playing field with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public static final Vector2 SILVER_DELIVERY_BUCKET_TARGET_POSITION = new Vector2(
            -4.5, //-5.0, //-5.5,
            -5.0
    );
    public static final Vector2 GOLD_DELIVERY_BUCKET_TARGET_POSIITON = new Vector2(
            4.5, //5.0, //5.5,
            -5.0
    );

    // In inches
    private static final double DELIVER_MINERALS_BUCKET_POSITION_TOLERANCE = 4.0;
    private static final double DELIVER_SILVER_THEN_GOLD_OPEN_BUCKET_POSITION_TOLERANCE = 20.0; //36.0; //12.0;
    private static final double DELIVER_GOLD_ONLY_OPEN_BUCKET_POSITION_TOLERANCE = 15.0; //20.0; //36.0; //12.0;
    private static final double DELIVER_SLIVER_ONLY_OPEN_BUCKET_POSITION_TOLERANCE = 15.0; //20.0; //36.0; //12.0;

    // In seconds
    private static final double DELIVER_SILVER_THEN_GOLD_DELAY_AFTER_OPEN_BUCKET = 0.5;
    private static final double DELIVER_GOLD_ONLY_DELAY_AFTER_OPEN_BUCKET = 0.5; //0.4; //0.5; //0.8;
    private static final double DELIVER_SILVER_ONLY_DELAY_AFTER_OPEN_BUCKET = 0.5; //0.4; //0.5; //0.8;

    // In inches per second
    private static final double DELIVER_MINERALS_BUCKET_SLIDE_TARGET_VELOCITY_DURING_DELAY_AFTER_COMPLETELY_OPEN_BUCKET = -35.0; //-30.0;

    // Between [0, 1] of bucket pivot target position used when bucket is in front of robot
    private static final double DISENGAGE_LATCH_BUCKET_PIVOT_TARGET_POSITION_WHEN_SLIDE_EXTENDED_FORWARD = 0.5;

    private static final double DISENGAGE_LATCH_NAV_TARGET_HEADING_OFFSET_FOR_CLEARING_HOOK = 10.0;

    // Between [0, 1] of bucket pivot target position
    private static final double CLEAR_MINERALS_OFF_ROBOT_BUCKET_PIVOT_TARGET_POSITION = 0.44; //0.435;

    // Between [0, 1] of bucket pivot target position
    private static final double CLEAR_MINERALS_OFF_ROBOT_BUCKET_PIVOT_POSITION_TOLERANCE = 0.02; //0.015;

    public enum Mode {
        NONE,

        // Does not continue
        PICKUP_MINERALS_WITHOUT_DELIVERY,

        // Continues to DELIVER_SILVER_THEN_GOLD
        PICKUP_MINERALS_WITH_DELIVERY,

        // Continues to DELIVER_GOLD_ONLY
        DELIVER_SILVER_THEN_GOLD,

        // Continues to PICKUP_MINERALS_WITH_DELIVERY
        DELIVER_GOLD_ONLY,

        // Continues to PICKUP_MINERALS_WITH_DELIVERY
        DELIVER_SILVER_ONLY,

        PREPARE_TO_DELIVER_GOLD,

        PREPARE_TO_DELIVER_SILVER,

        // Does not continue
        DISENGAGE_LATCH,

        // Continues to STOP_LATCH
        ENGAGE_LATCH,

        // Does not continue
        STOP_LATCH,

        // Does not continue
        CLEAR_MINERALS_OFF_ROBOT;

        public boolean isPickupMinerals() {
            return this == PICKUP_MINERALS_WITHOUT_DELIVERY || this == PICKUP_MINERALS_WITH_DELIVERY;
        }

        public boolean isDeliverMinerals() {
            return this == DELIVER_SILVER_THEN_GOLD || this == DELIVER_GOLD_ONLY || this == DELIVER_SILVER_ONLY;
        }

        public boolean isLatch() {
            return this == DISENGAGE_LATCH || this == ENGAGE_LATCH || this == STOP_LATCH;
        }
    }

    // See getters/setters
    private Mode mode = Mode.NONE;
    private Vector2 leftCraterMineralPickupPosition = new Vector2(-70.0, -70.0);
    private Vector2 rightCraterMineralPickupPosition = new Vector2(70.0, 70.0);
    private static double latchNavTargetHeading = 135.0;

    private double mineralDeliveryBucketOpenTime = 0.0;

    // Automator needs access to Nav, Bucket, and Latch
    private final Nav nav;
    private final Bucket bucket;
    private final Latch latch;

    public Automator(Telemetry telemetry, HardwareMap hardwareMap, Nav nav, Bucket bucket, Latch latch) {
        super(telemetry, hardwareMap);

        this.nav = nav;
        this.bucket = bucket;
        this.latch = latch;
    }

    public Automator(Telemetry telemetry, HardwareMap hardwareMap, Nav nav, Bucket bucket, Latch latch, double disengageLatchNavTargetHeading) {
        this(telemetry, hardwareMap, nav, bucket, latch);

        setDisengageLatchNavTargetHeading(disengageLatchNavTargetHeading);
    }

    // In degrees starting with robot facing positive x-axis (facing right from the the driving team's perspective) going counterclockwise
    public double getDisengageLatchNavTargetHeading() {
        return latchNavTargetHeading;
    }

    // In degrees starting with robot facing positive x-axis (facing right from the the driving team's perspective) going counterclockwise
    public void setDisengageLatchNavTargetHeading(double disengageLatchNavTargetHeading) {
        Automator.latchNavTargetHeading = disengageLatchNavTargetHeading;
    }

    public Vector2 getMineralPickupPosition() {
        return nav.getPosition().getX() < 0.0 ? leftCraterMineralPickupPosition : rightCraterMineralPickupPosition;
    }

    public void setMineralPickupPosition(Vector2 mineralPickupPosition) {
        if (nav.getPosition().getX() < 0.0) {
            leftCraterMineralPickupPosition = mineralPickupPosition;
        } else {
            rightCraterMineralPickupPosition = mineralPickupPosition;
        }
    }

    public Mode getMode() {
        return mode;
    }

    // If mode is switching to a delivery mode and bucket slide is extended forward, will also setMineralPickupPosition()
    public void setMode(Mode mode) {
        if (mode.isDeliverMinerals() && !this.mode.isDeliverMinerals() && bucket.getRelativePosition() > Bucket.MAX_UNEXTENDED_SLIDE_POSITION) {
            setMineralPickupPosition(bucket.getPosition());
        }

        if (mode != this.mode) mineralDeliveryBucketOpenTime = 0.0;

        this.mode = mode;
    }

    private void setNavTargetVelocityAndRotateTowardsHeading(Vector2 navTargetVelocity, double navTargetHeading) {
        nav.setTargetVelocities(navTargetVelocity, Degrees.normalize(navTargetHeading - nav.getHeading()) * NAV_HEADING_CORRECTION_GAIN);
    }

    public void setTargets(Vector2 navTargetVelocity, double defaultNavTargetAngularVelocity) {
        switch (mode) {
            case PICKUP_MINERALS_WITHOUT_DELIVERY: {
                pickupMineralsWithoutDelivery(navTargetVelocity, defaultNavTargetAngularVelocity);
                break;
            }
            case PICKUP_MINERALS_WITH_DELIVERY: {
                pickupMineralsWithDelivery(navTargetVelocity, defaultNavTargetAngularVelocity);
                break;
            }
            case DELIVER_SILVER_THEN_GOLD: {
                deliverMinerals(navTargetVelocity, SILVER_DELIVERY_BUCKET_TARGET_POSITION, DELIVER_SILVER_THEN_GOLD_OPEN_BUCKET_POSITION_TOLERANCE,
                        SILVER_ONLY_DELIVERY_BUCKET_TENSIONER_TARGET_POSITION);
                break;
            }
            case DELIVER_GOLD_ONLY: {
                deliverMinerals(navTargetVelocity, GOLD_DELIVERY_BUCKET_TARGET_POSIITON, DELIVER_GOLD_ONLY_OPEN_BUCKET_POSITION_TOLERANCE,
                        bucket.tensioner.getMinPosition());
                break;
            }
            case DELIVER_SILVER_ONLY: {
                deliverMinerals(navTargetVelocity, SILVER_DELIVERY_BUCKET_TARGET_POSITION, DELIVER_SLIVER_ONLY_OPEN_BUCKET_POSITION_TOLERANCE,
                        bucket.tensioner.getMinPosition());
                break;
            }
            case PREPARE_TO_DELIVER_GOLD: {
                prepareToDeliverMinerals(navTargetVelocity, GOLD_DELIVERY_BUCKET_TARGET_POSIITON);
                break;
            }
            case PREPARE_TO_DELIVER_SILVER: {
                prepareToDeliverMinerals(navTargetVelocity, SILVER_DELIVERY_BUCKET_TARGET_POSITION);
                break;
            }
            case DISENGAGE_LATCH: {
                disengageLatch(navTargetVelocity, defaultNavTargetAngularVelocity);
                break;
            }
            case ENGAGE_LATCH: {
                engageLatch(navTargetVelocity, defaultNavTargetAngularVelocity);
                nav.setTargetVelocities(navTargetVelocity, defaultNavTargetAngularVelocity);
                break;
            }
            case STOP_LATCH: {
                stopLatch(navTargetVelocity, defaultNavTargetAngularVelocity);
                nav.setTargetVelocities(navTargetVelocity, defaultNavTargetAngularVelocity);
                break;
            }
            case CLEAR_MINERALS_OFF_ROBOT: {
                clearMineralsOffRobot(navTargetVelocity, defaultNavTargetAngularVelocity);
            }
        }
    }

    private void pickupMineralsWithoutDelivery(Vector2 navTargetVelocity, double navTargetAngularVelocity) {
        if (bucket.getSlidePosition() < Bucket.MIN_UNEXTENDED_SLIDE_POSITION) {
            nav.setTargetVelocities(navTargetVelocity, navTargetAngularVelocity);
        } else {
            setNavTargetVelocityAndRotateTowardsHeading(navTargetVelocity, getMineralPickupPosition().getRotationFrom(nav.getPosition()));
        }

        if (bucket.getSlidePosition() < Bucket.MIN_UNEXTENDED_SLIDE_POSITION) {
            bucket.pivot.setTargetPosition(bucket.pivot.getMaxPosition());
        } else {
            bucket.setPivotTargetPosition();
        }

        double slideTargetPosition = Bucket.convertRelativePositionToSlidePosition(nav.getPosition().getDistanceFrom(getMineralPickupPosition()));
        bucket.leftSlide.setTargetPosition(slideTargetPosition);
        bucket.rightSlide.setTargetPosition(slideTargetPosition);

        bucket.setTensionerTargetPosition();
    }

    private void pickupMineralsWithDelivery(Vector2 navTargetVelocity, double navTargetAngularVelocity) {
        if (bucket.getSlidePosition() < Bucket.MIN_UNEXTENDED_SLIDE_POSITION || navTargetAngularVelocity != 0.0) {
            nav.setTargetVelocities(navTargetVelocity, navTargetAngularVelocity);
        } else {
            setNavTargetVelocityAndRotateTowardsHeading(navTargetVelocity, getMineralPickupPosition().getRotationFrom(nav.getPosition()));
        }

        if (bucket.getSlidePosition() < Bucket.MIN_UNEXTENDED_SLIDE_POSITION) {
            bucket.pivot.setTargetPosition(bucket.pivot.getMaxPosition());
        } else {
            bucket.setPivotTargetPosition();
        }

        double slidePower = bucket.getSlidePosition() < Bucket.MAX_UNEXTENDED_SLIDE_POSITION ? 1.0 : MINERAL_PICKUP_WITH_DELIVERY_BUCKET_EXTENDED_FORWARD_SLIDE_POWER;
        bucket.leftSlide.setPower(slidePower);
        bucket.rightSlide.setPower(slidePower);

        bucket.setTensionerTargetPosition();
    }

    private void deliverMinerals(Vector2 navTargetVelocity, Vector2 bucketTargetPosition, double openBucketPositionTolerance, double openTensionerPosition) {
        bucket.setPivotTargetPosition();

        if (bucket.getSlidePosition() > Bucket.MAX_UNEXTENDED_SLIDE_POSITION) {
            setNavTargetVelocityAndRotateTowardsHeading(navTargetVelocity, getMineralPickupPosition().getRotationFrom(nav.getPosition()));

            bucket.tensioner.setTargetPosition(bucket.tensioner.getMaxPosition());

            if (bucket.tensioner.isPositionGreaterThanOrAt(MINERAL_DELIVERY_MIN_CLOSED_BUCKET_TENSIONER_POSITION)) {
                bucket.leftSlide.setPower(-1.0);
                bucket.rightSlide.setPower(-1.0);
            } else {
                bucket.leftSlide.setPower(0.0);
                bucket.rightSlide.setPower(0.0);
            }
        } else {
            setNavTargetVelocityAndRotateTowardsHeading(
                    navTargetVelocity,
                    bucketTargetPosition.getRotationFrom(nav.getPosition()) + 180.0
            );

            double tensionerTargetPosition;
            if (mineralDeliveryBucketOpenTime == 0.0 || openTensionerPosition > 0.0) {
                double slideTargetPosition = Bucket.convertRelativePositionToSlidePosition(-nav.getPosition().getDistanceFrom(bucketTargetPosition));

                if (bucket.getSlidePosition() > Bucket.MINERAL_DELIVERY_HEIGHT_MAX_SLIDE_POSITION ||
                        slideTargetPosition - bucket.getSlidePosition() > DELIVER_MINERALS_BUCKET_POSITION_TOLERANCE ||
                        (slideTargetPosition - bucket.getSlidePosition() < -DELIVER_MINERALS_BUCKET_POSITION_TOLERANCE &&
                                bucket.leftSlide.getVelocity() > DELIVER_MINERALS_MIN_BUCKET_SLIDE_VELOCITY_FOR_ACCIDENTALLY_JERKING_MINERALS_OUT)) {
                    tensionerTargetPosition = bucket.tensioner.getMaxPosition();

                    if (bucket.tensioner.getPosition() > DELIVER_MINERALS_MAX_BUCKET_TENSIONER_POSITION_FOR_ACCIDENTALLY_JERKING_MINERALS_OUT) {
                        bucket.leftSlide.setTargetPosition(slideTargetPosition);
                        bucket.rightSlide.setTargetPosition(slideTargetPosition);
                    } else {
                        bucket.leftSlide.setPower(0.0);
                        bucket.rightSlide.setPower(0.0);
                    }
                } else {
                    bucket.leftSlide.setTargetPosition(slideTargetPosition);
                    bucket.rightSlide.setTargetPosition(slideTargetPosition);

                    tensionerTargetPosition = Range.clip(Range.scale(
                            bucket.getPosition().getDistanceFrom(bucketTargetPosition),
                            DELIVER_MINERALS_BUCKET_POSITION_TOLERANCE, openBucketPositionTolerance,
                            openTensionerPosition, bucket.tensioner.getMaxPosition()
                    ), openTensionerPosition, bucket.tensioner.getMaxPosition());
                }
            } else {
                // Jerk minerals out of bucket while the bucket is completely open
                bucket.leftSlide.setTargetPosition(bucket.leftSlide.getMinPosition(), DELIVER_MINERALS_BUCKET_SLIDE_TARGET_VELOCITY_DURING_DELAY_AFTER_COMPLETELY_OPEN_BUCKET);
                bucket.rightSlide.setTargetPosition(bucket.rightSlide.getMinPosition(), DELIVER_MINERALS_BUCKET_SLIDE_TARGET_VELOCITY_DURING_DELAY_AFTER_COMPLETELY_OPEN_BUCKET);

                tensionerTargetPosition = bucket.tensioner.getMinPosition();
            }

            if (openTensionerPosition > bucket.tensioner.getMinPosition()) {
                bucket.tensioner.setTargetPosition(tensionerTargetPosition, SILVER_ONLY_DELIVERY_BUCKET_TENSIONER_MAX_VELOCITY);
            } else {
                bucket.tensioner.setTargetPosition(tensionerTargetPosition, BUCKET_TENSIONER_MAX_VELOCITY);
            }

            /*
            if (mineralDeliveryBucketOpenTime > 0.0) {
                // Jerk mineral out of bucket while the bucket is open
                bucket.leftSlide.setPower(-1.0);
                bucket.rightSlide.setPower(-1.0);
                bucket.tensioner.setTargetPosition(openTensionerPosition);
            } else {
                double slideTargetPosition = Bucket.convertRelativePositionToSlidePosition(-nav.getPosition().getDistanceFrom(bucketTargetPosition));
                bucket.leftSlide.setTargetPosition(slideTargetPosition);
                bucket.rightSlide.setTargetPosition(slideTargetPosition);

                if (slideTargetPosition - bucket.getSlidePosition() < -DELIVER_MINERALS_BUCKET_POSITION_TOLERANCE &&
                        bucket.leftSlide.getVelocity() > DELIVER_MINERALS_MIN_BUCKET_SLIDE_VELOCITY_FOR_ACCIDENTALLY_JERKING_MINERALS_OUT) {
                    if (bucket.tensioner.isPositionAtMax()) {
                        bucket.tensioner.setTargetPosition(bucket.tensioner.getMaxPosition());
                    } else {
                        bucket.tensioner.setPower(1.0);
                    }
                } else {
                    double tensionerTargetPosition = Range.clip(Range.scale(
                            bucket.getPosition().getDistanceFrom(bucketTargetPosition),
                            DELIVER_MINERALS_BUCKET_POSITION_TOLERANCE, openBucketPositionTolerance,
                            openTensionerPosition, bucket.tensioner.getMaxPosition()
                    ), openTensionerPosition, bucket.tensioner.getMaxPosition());

                    if (tensionerTargetPosition == bucket.tensioner.getMinPosition() && !bucket.tensioner.isPositionLessThanOrAt(bucket.tensioner.getMinPosition())) {
                        bucket.tensioner.setPower(COMPLETELY_OPEN_BUCKET_TENSIONER_POWER);
                    } else {
                        bucket.tensioner.setTargetPosition(tensionerTargetPosition);
                    }
                }
            }
            */
        }
    }

    private void prepareToDeliverMinerals(Vector2 navTargetVelocity, Vector2 bucketTargetPosition) {
        bucket.setPivotTargetPosition();

        if (bucket.getSlidePosition() > Bucket.MAX_UNEXTENDED_SLIDE_POSITION) {
            setNavTargetVelocityAndRotateTowardsHeading(navTargetVelocity, getMineralPickupPosition().getRotationFrom(nav.getPosition()));

            bucket.tensioner.setTargetPosition(bucket.tensioner.getMaxPosition());

            if (bucket.tensioner.isPositionGreaterThanOrAt(MINERAL_DELIVERY_MIN_CLOSED_BUCKET_TENSIONER_POSITION)) {
                bucket.leftSlide.setPower(-1.0);
                bucket.rightSlide.setPower(-1.0);
            } else {
                bucket.leftSlide.setPower(0.0);
                bucket.rightSlide.setPower(0.0);
            }
        } else {
            setNavTargetVelocityAndRotateTowardsHeading(
                    navTargetVelocity,
                    bucketTargetPosition.getRotationFrom(nav.getPosition()) + 180.0
            );

            double slideTargetPosition = Bucket.convertRelativePositionToSlidePosition(-nav.getPosition().getDistanceFrom(bucketTargetPosition));
            bucket.leftSlide.setTargetPosition(slideTargetPosition);
            bucket.rightSlide.setTargetPosition(slideTargetPosition);

            bucket.tensioner.setTargetPosition(bucket.tensioner.getMaxPosition());
        }
    }

    private void disengageLatch(Vector2 navTargetVelocity, double defaultNavTargetAngularVelocity) {
        if (bucket.getSlidePosition() > Bucket.MIN_UNEXTENDED_SLIDE_POSITION && bucket.getSlidePosition() < Bucket.MAX_UNEXTENDED_SLIDE_POSITION &&
                defaultNavTargetAngularVelocity == 0.0) {
            setNavTargetVelocityAndRotateTowardsHeading(
                    navTargetVelocity,
                    latchNavTargetHeading - DISENGAGE_LATCH_NAV_TARGET_HEADING_OFFSET_FOR_CLEARING_HOOK
            );
        } else {
            nav.setTargetVelocities(navTargetVelocity, defaultNavTargetAngularVelocity);
        }

        bucket.leftSlide.setTargetPosition(bucket.leftSlide.getInitialPosition());
        bucket.rightSlide.setTargetPosition(bucket.rightSlide.getInitialPosition());
        bucket.tensioner.setTargetPosition(bucket.tensioner.getMaxPosition());

        if (bucket.leftSlide.getPosition() < Bucket.MAX_UNEXTENDED_SLIDE_POSITION || latch.getCatchEngagementAmount() > 0.0) {
            latch.disengage();
        } else {
            bucket.pivot.setTargetPosition(DISENGAGE_LATCH_BUCKET_PIVOT_TARGET_POSITION_WHEN_SLIDE_EXTENDED_FORWARD);
        }
    }

    private void engageLatch(Vector2 navTargetVelocity, double navTargetAngularVelocity) {
        nav.setTargetVelocities(navTargetVelocity, navTargetAngularVelocity);
        bucket.leftSlide.brake();
        bucket.rightSlide.brake();
        bucket.tensioner.setTargetPosition(bucket.tensioner.getMaxPosition());
        latch.engage();
    }

    private void stopLatch(Vector2 navTargetVelocity, double navTargetAngularVelocity) {
        nav.setTargetVelocities(navTargetVelocity, navTargetAngularVelocity);
        bucket.leftSlide.brake();
        bucket.rightSlide.brake();
        bucket.tensioner.setTargetPosition(bucket.tensioner.getMaxPosition());
        latch.stop();
    }

    private void clearMineralsOffRobot(Vector2 navTargetVelocity, double navTargetAngularVelocity) {
        nav.setTargetVelocities(navTargetVelocity, navTargetAngularVelocity);

        bucket.tensioner.setTargetPosition(bucket.tensioner.getMaxPosition());

        if (bucket.leftSlide.isPositionLessThanOrAt(Bucket.MIN_UNEXTENDED_SLIDE_POSITION)) {
            bucket.pivot.setTargetPosition(bucket.pivot.getMaxPosition());
        } else if (!bucket.leftSlide.isPositionLessThanOrAt(Bucket.OPEN_BUCKET_FOR_MINERAL_PICKUP_MIN_SLIDE_POSITION)) {
            bucket.pivot.setTargetPosition(CLEAR_MINERALS_OFF_ROBOT_BUCKET_PIVOT_TARGET_POSITION);
        }

        if (bucket.pivot.isPositionAt(CLEAR_MINERALS_OFF_ROBOT_BUCKET_PIVOT_TARGET_POSITION, CLEAR_MINERALS_OFF_ROBOT_BUCKET_PIVOT_POSITION_TOLERANCE)) {
            bucket.leftSlide.setTargetPosition(Bucket.MIN_UNEXTENDED_SLIDE_POSITION);
            bucket.rightSlide.setTargetPosition(Bucket.MIN_UNEXTENDED_SLIDE_POSITION);
        } else {
            bucket.leftSlide.setTargetPosition(Bucket.OPEN_BUCKET_FOR_MINERAL_PICKUP_MIN_SLIDE_POSITION);
            bucket.rightSlide.setTargetPosition(Bucket.OPEN_BUCKET_FOR_MINERAL_PICKUP_MIN_SLIDE_POSITION);
        }
    }

    // Called through Component.update()
    @Override
    void internalUpdate() {
        switch (mode) {
            case PICKUP_MINERALS_WITH_DELIVERY: {
                if (bucket.leftSlide.getPosition() > Bucket.MAX_UNEXTENDED_SLIDE_POSITION &&
                        (bucket.leftSlide.isDrivingForward() || bucket.leftSlide.isPositionAtMax()) &&
                        (bucket.rightSlide.isDrivingForward() || bucket.rightSlide.isPositionAtMax()) &&
                        bucket.leftSlide.getVelocity() < MINERAL_PICKUP_WITH_DELIVERY_FINISHED_MAX_BUCKET_SLIDE_VELOCITY &&
                        bucket.leftSlide.getAcceleration() < 0.0) {
                    setMode(Mode.DELIVER_SILVER_THEN_GOLD);
                }
                break;
            }
            case DELIVER_SILVER_THEN_GOLD: {
                if (bucket.getSlidePosition() < Bucket.MIN_UNEXTENDED_SLIDE_POSITION &&
                        bucket.tensioner.isPositionLessThanOrAt(SILVER_ONLY_DELIVERY_BUCKET_TENSIONER_TARGET_POSITION)) {
                    mineralDeliveryBucketOpenTime += deltaTime;

                    if (mineralDeliveryBucketOpenTime > DELIVER_SILVER_THEN_GOLD_DELAY_AFTER_OPEN_BUCKET ||
                            nav.getPosition().getDistanceFrom(SILVER_DELIVERY_BUCKET_TARGET_POSITION) < nav.getPosition().getDistanceFrom(GOLD_DELIVERY_BUCKET_TARGET_POSIITON)) {
                        setMode(Mode.DELIVER_GOLD_ONLY);
                    }
                } else {
                    mineralDeliveryBucketOpenTime = 0.0;
                }
                break;
            }
            case DELIVER_GOLD_ONLY: {
                if (bucket.getSlidePosition() < Bucket.MIN_UNEXTENDED_SLIDE_POSITION &&
                        bucket.tensioner.isPositionLessThanOrAt(bucket.tensioner.getMinPosition())) {
                    mineralDeliveryBucketOpenTime += deltaTime;

                    if (mineralDeliveryBucketOpenTime > DELIVER_GOLD_ONLY_DELAY_AFTER_OPEN_BUCKET) {
                        setMode(Mode.PICKUP_MINERALS_WITH_DELIVERY);
                    }
                } else {
                    mineralDeliveryBucketOpenTime = 0.0;
                }
                break;
            }
            case DELIVER_SILVER_ONLY: {
                if (bucket.getSlidePosition() < Bucket.MIN_UNEXTENDED_SLIDE_POSITION &&
                        bucket.tensioner.isPositionLessThanOrAt(SILVER_ONLY_DELIVERY_BUCKET_TENSIONER_TARGET_POSITION)) {
                    mineralDeliveryBucketOpenTime += deltaTime;

                    if (mineralDeliveryBucketOpenTime > DELIVER_SILVER_ONLY_DELAY_AFTER_OPEN_BUCKET) {
                        setMode(Mode.PICKUP_MINERALS_WITH_DELIVERY);
                    }
                } else {
                    mineralDeliveryBucketOpenTime = 0.0;
                }
                break;
            }
            case ENGAGE_LATCH: {
                if (latch.isEngaged()) setMode(Mode.STOP_LATCH);
                break;
            }
        }
    }

    @Override
    public String toString() {
        return createStateString("mode", getMode()) +
                createStateString("mineralPickupPosition", getMineralPickupPosition().toString("%.2fin")) +
                createStateString("leftCraterMineralPickupPosition", leftCraterMineralPickupPosition.toString("%.2fin")) +
                createStateString("rightCraterMineralPickupPosition", rightCraterMineralPickupPosition.toString("%.2fin"));
    }
}
